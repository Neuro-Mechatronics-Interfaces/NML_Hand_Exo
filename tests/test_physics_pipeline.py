from __future__ import annotations

import json

import numpy as np
import pytest

from physics_pipeline.contracts import (
    ExoMotorDescriptor,
    build_exo_state_channels,
)
from physics_pipeline.evaluation import evaluate_intent_grouped, regression_metrics
from physics_pipeline.geometry import (
    GeneralizedCoordinate,
    MotorCoordinateMapping,
    ReducedGeometry,
)
from physics_pipeline.manifest import SessionManifest
from physics_pipeline.markers import format_marker, parse_marker
from physics_pipeline.models import (
    ActivationDynamics,
    LinearStateSpaceModel,
    ReducedPhysicsModel,
    ReducedPhysicsParameters,
    StateConditionedIntentModel,
)
from physics_pipeline.xdf_import import (
    PhysicsSession,
    align_numeric_stream,
    align_step_stream,
    count_invalid_event_json,
    extract_event_stream,
)
from physics_pipeline.xdf_inspect import summarize_stream
from physics_pipeline.xdf_replay import ReplayStream, build_event_schedule, _channel_info
from nml_hand_exo.interface._command_stream import (
    CommandMotor,
    CommandStateTracker,
    command_channel_specs,
)
from nml_hand_exo.interface._telemetry_streaming import (
    StringLSLEventOutlet,
    StructuredLSLTelemetryOutlet,
)


def test_exo_state_contract_is_unambiguous_and_stable():
    channels = build_exo_state_channels(
        [
            ExoMotorDescriptor(6, "index", "left"),
            ExoMotorDescriptor(16, "index", "right"),
        ]
    )
    labels = [channel.label for channel in channels]
    assert labels[:3] == [
        "frame.sequence",
        "frame.firmware_timestamp_ms",
        "frame.fast_read_flags",
    ]
    assert "L.index.id6.present_current_mA" in labels
    assert "R.index.id16.estimated_motor_torque_from_current_Nm" in labels
    assert len(labels) == len(set(labels))


def test_exo_state_contract_rejects_duplicate_ids():
    with pytest.raises(ValueError, match="duplicate"):
        build_exo_state_channels(
            [ExoMotorDescriptor(16, "index", "right"), ExoMotorDescriptor(16, "other", "right")]
        )


def test_structured_lsl_outlet_retains_disabled_contract_without_importing_lsl():
    outlet = StructuredLSLTelemetryOutlet("state", "State", "source", "schema.v1")
    outlet.configure(
        False,
        [
            {"label": "a", "unit": "degrees", "quantity": "angle"},
            {"label": "b", "unit": "mA", "quantity": "current"},
        ],
        nominal_srate=50,
    )
    assert outlet.channel_labels == ("a", "b")
    assert outlet.enabled is False


def test_string_lsl_event_outlet_can_be_disabled_without_importing_lsl():
    outlet = StringLSLEventOutlet("events", "Events", "source", "schema.v1")
    outlet.configure(False)
    outlet.publish({"event": "ignored"})
    assert outlet.enabled is False


def test_lsl_publish_failures_are_recording_errors_not_control_exceptions():
    class BrokenOutlet:
        def push_sample(self, *_args):
            raise RuntimeError("LSL write failed")

    numeric = StructuredLSLTelemetryOutlet("state", "State", "source", "schema.v1")
    numeric.enabled = True
    numeric._channel_specs = [{"label": "a"}]
    numeric._outlet = BrokenOutlet()
    numeric.publish({"a": 1.0})
    assert numeric.last_error == "LSL write failed"

    events = StringLSLEventOutlet("events", "Events", "source", "schema.v1")
    events.enabled = True
    events._outlet = BrokenOutlet()
    events.publish({"value": float("nan")})
    assert events.last_error


def test_command_contract_and_tracker_preserve_unknown_firmware_goals():
    motors = [
        CommandMotor(6, "index", "L"),
        CommandMotor(16, "index", "R"),
    ]
    specs = command_channel_specs(motors)
    labels = [str(spec["label"]) for spec in specs]
    assert len(labels) == len(set(labels))
    assert "R.index.id16.requested_current_mA" in labels

    tracker = CommandStateTracker()
    tracker.configure_motors(motors)
    tracker.observe(
        {"command": "set_control_mode:all:current", "status": "sent", "source": "gui"}
    )
    tracker.observe(
        {"command": "set_current:16:75", "status": "sent", "source": "emg"}
    )
    before_gesture = tracker.snapshot()
    tracker.observe(
        {"command": "set_gesture:index:flex", "status": "sent", "source": "udp"}
    )
    after_gesture = tracker.snapshot()
    assert before_gesture["global.control_mode_request_code"] == 4
    assert before_gesture["R.index.id16.requested_current_mA"] == pytest.approx(75)
    assert before_gesture["R.index.id16.requested_relative_angle_deg"] is None
    assert after_gesture["R.index.id16.requested_relative_angle_deg"] is None
    assert after_gesture["global.command_source_code"] == 2


def test_command_tracker_updates_only_transmitted_requests():
    tracker = CommandStateTracker()
    tracker.configure_motors([CommandMotor(16, "index", "R")])
    tracker.observe(
        {"command": "set_current:16:80", "status": "failed", "source": "emg"}
    )
    assert tracker.snapshot()["R.index.id16.requested_current_mA"] is None
    tracker.observe(
        {"command": "set_current:16:80", "status": "sent", "source": "emg"}
    )
    tracker.observe(
        {"command": "stop:16", "status": "sent", "source": "safety"}
    )
    snapshot = tracker.snapshot()
    assert snapshot["R.index.id16.requested_current_mA"] == 0.0
    assert snapshot["R.index.id16.direct_command_active"] is False


def test_marker_roundtrip_preserves_condition_fields():
    marker = format_marker(
        "prompt_onset",
        {"phase": "gesture", "gesture": "attempt_close", "condition": "exo_transparent"},
    )
    assert parse_marker(marker) == {
        "event": "prompt_onset",
        "phase": "gesture",
        "gesture": "attempt_close",
        "condition": "exo_transparent",
    }


def test_manifest_roundtrip(tmp_path):
    path = tmp_path / "session.json"
    SessionManifest(session_id="s001", side="right", condition="exo_transparent").save(path)
    loaded = SessionManifest.load(path)
    assert loaded.session_id == "s001"
    assert loaded.schema == "nml.physics_session.v1"


def test_numeric_alignment_reports_stale_samples():
    stream = {
        "time_stamps": np.asarray([0.0, 1.0, 2.0]),
        "time_series": np.asarray([[0.0], [10.0], [20.0]]),
        "info": {"desc": [None]},
    }
    aligned, age, valid, labels = align_numeric_stream(
        stream, np.asarray([0.05, 0.5, 1.95, 3.0]), max_age_s=0.1
    )
    assert labels == ("ch0",)
    assert valid.tolist() == [True, False, True, False]
    assert aligned[0, 0] == pytest.approx(0.5)
    assert np.isnan(aligned[1, 0])
    assert age[2] == pytest.approx(0.05)


def test_command_alignment_uses_latest_preceding_snapshot_and_allows_nan_fields():
    stream = {
        "time_stamps": np.asarray([1.0, 2.0, 2.0, 3.0]),
        "time_series": np.asarray(
            [[1.0, np.nan], [2.0, np.nan], [20.0, np.nan], [3.0, 30.0]]
        ),
        "info": {"desc": [None]},
    }
    aligned, age, valid, labels = align_step_stream(
        stream, np.asarray([0.5, 1.5, 2.0, 2.9, 3.3]), max_age_s=1.0
    )
    assert labels == ("ch0", "ch1")
    assert valid.tolist() == [False, True, True, True, True]
    assert aligned[1, 0] == 1.0
    assert aligned[2, 0] == 20.0
    assert np.isnan(aligned[2, 1])
    assert age[-1] == pytest.approx(0.3)


def test_event_extraction_preserves_json_and_timestamps():
    stream = {
        "time_stamps": np.asarray([1.25, 2.5]),
        "time_series": np.asarray([["{\"status\":\"sent\"}"], ["{\"status\":\"failed\"}"]]),
    }
    timestamps, events = extract_event_stream(stream)
    np.testing.assert_allclose(timestamps, [1.25, 2.5])
    assert json.loads(events[1])["status"] == "failed"
    assert count_invalid_event_json(events) == 0
    assert count_invalid_event_json(np.asarray(["[]", "not-json"])) == 2


def test_physics_session_roundtrip_preserves_commands_and_events(tmp_path):
    session = PhysicsSession(
        timestamps=np.asarray([1.0]),
        emg_windows=np.zeros((1, 2, 4), dtype=np.float32),
        emg_rms=np.zeros((1, 2)),
        labels=np.asarray(["rest"]),
        trials=np.asarray(["trial-1"]),
        marker_json=np.asarray(["{}"]),
        exo_state=np.empty((1, 0)),
        exo_state_age_s=np.asarray([np.inf]),
        exo_state_valid=np.asarray([False]),
        exo_command=np.asarray([[1.0, np.nan]]),
        exo_command_age_s=np.asarray([0.01]),
        exo_command_valid=np.asarray([True]),
        exo_command_channels=("frame.sequence", "unknown"),
        exo_event_timestamps=np.asarray([0.9]),
        exo_event_json=np.asarray(['{"status":"sent"}']),
        metadata={"schema": "nml.physics_session.v1"},
    )
    path = tmp_path / "session.npz"
    session.save(path)
    loaded = PhysicsSession.load(path)
    assert loaded.exo_command_valid.tolist() == [True]
    assert np.isnan(loaded.exo_command[0, 1])
    assert json.loads(loaded.exo_event_json[0])["status"] == "sent"


def test_xdf_stream_summary_handles_missing_metadata():
    stream = {
        "time_series": np.ones((5, 2)),
        "time_stamps": np.arange(5) / 10.0,
        "info": {
            "name": ["Signal"],
            "type": ["EMG"],
            "source_id": ["source"],
            "nominal_srate": ["10"],
            "channel_format": ["float32"],
            "desc": [None],
        },
    }
    summary = summarize_stream(stream)
    assert summary["channel_labels"] == ["ch0", "ch1"]
    assert summary["nonmonotonic_intervals"] == 0
    assert summary["median_effective_srate_hz"] == pytest.approx(10.0)


def test_replay_schedule_preserves_cross_stream_order():
    info = {}
    streams = [
        ReplayStream("a", "EMG", "a", 10, "float32", np.zeros((2, 1)), np.asarray([5.0, 5.2]), info),
        ReplayStream("b", "Markers", "b", 0, "string", np.asarray([["x"]]), np.asarray([5.1]), info),
    ]
    schedule = build_event_schedule(streams)
    ordered = [schedule[0]]
    import heapq

    ordered = [heapq.heappop(schedule) for _ in range(len(schedule))]
    assert [item[0] for item in ordered] == pytest.approx([0.0, 0.1, 0.2])


def test_split_replay_channel_metadata_has_explicit_labels():
    info = _channel_info([("EMG1", "uV", "emg"), ("EMG2", "uV", "emg")])
    channels = info["desc"][0]["channels"][0]["channel"]
    assert [channel["label"][0] for channel in channels] == ["EMG1", "EMG2"]


def test_activation_dynamics_rises_and_relaxes():
    excitation = np.asarray([0.0, 1.0, 1.0, 0.0, 0.0])
    activation = ActivationDynamics().filter(excitation, 0.05).ravel()
    assert activation[2] > activation[1] > activation[0]
    assert activation[4] < activation[3] < activation[2]


def test_state_conditioned_intent_model_and_grouped_evaluation():
    rng = np.random.default_rng(4)
    labels = np.asarray([label for group in range(6) for label in ("open", "close") for _ in range(8)])
    groups = np.asarray([f"g{group}-{label}" for group in range(6) for label in ("open", "close") for _ in range(8)])
    emg = rng.normal(scale=0.2, size=(len(labels), 4))
    emg[labels == "close", 0] += 2.0
    state = rng.normal(scale=0.1, size=(len(labels), 2))
    model = StateConditionedIntentModel().fit(emg, labels, state)
    assert np.mean(model.predict(emg, state) == labels) > 0.95
    report = evaluate_intent_grouped(emg, labels, groups, state, folds=3)
    assert report["balanced_accuracy_mean"] > 0.9


def test_linear_state_space_recovers_known_dynamics():
    a = np.asarray([[0.9, 0.1], [0.0, 0.8]])
    b = np.asarray([[0.2], [0.1]])
    u = np.sin(np.linspace(0, 8, 300))[:, None]
    x = np.zeros((len(u), 2))
    for index in range(len(u) - 1):
        x[index + 1] = a @ x[index] + b @ u[index]
    model = LinearStateSpaceModel(regularization=1e-10).fit(x, u)
    assert model.A == pytest.approx(a, abs=1e-5)
    assert model.B == pytest.approx(b, abs=1e-5)


def test_reduced_physics_model_has_expected_torque_direction():
    params = ReducedPhysicsParameters(
        inertia=np.asarray([1.0]),
        stiffness=np.asarray([2.0]),
        damping=np.asarray([0.5]),
        rest_position=np.asarray([0.0]),
        emg_torque_map=np.asarray([[3.0]]),
        bias_torque=np.asarray([0.0]),
    )
    model = ReducedPhysicsModel(params)
    assert model.acceleration([0], [0], [1], [0])[0] == pytest.approx(3.0)
    assert model.acceleration([1], [0], [0], [0])[0] == pytest.approx(-2.0)


def test_geometry_requires_explicit_valid_mapping():
    geometry = ReducedGeometry(
        name="one_digit",
        side="right",
        coordinates=[GeneralizedCoordinate("index_flex")],
        mappings=[MotorCoordinateMapping(16, "index", "index_flex", 0.5, 1)],
    )
    np.testing.assert_allclose(geometry.motor_to_coordinate_matrix(), [[0.5]])
    with pytest.raises(ValueError, match="unknown coordinate"):
        ReducedGeometry(
            name="bad",
            side="right",
            coordinates=[GeneralizedCoordinate("index_flex")],
            mappings=[MotorCoordinateMapping(16, "index", "missing", 1.0, 1)],
        ).validate()


def test_regression_metrics_are_zero_error_for_identity():
    values = np.arange(12, dtype=float).reshape(6, 2)
    metrics = regression_metrics(values, values)
    assert metrics["mae_mean"] == 0.0
    assert metrics["r2_variance_weighted"] == 1.0
