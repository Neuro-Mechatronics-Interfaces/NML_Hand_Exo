"""Declared USB routes for every command handled by ``parseMessage()``.

The Protobuf firmware is deliberately a hybrid transport: bounded typed RPCs
use the secondary CDC, while maintenance, calibration, diagnostics and larger
text replies continue to use the primary CDC. Keep these sets synchronized
with ``utils.cpp`` and ``protocol/exo_usb.proto``; the protocol contract test
fails when a parser command is added without being classified here.
"""

from __future__ import annotations


# Existing text commands with a direct, typed handler in protobuf_usb.cpp.
PROTOBUF_RPC_COMMANDS = frozenset(
    {
        "assist_calibrate",
        "assist_config",
        "assist_enable",
        "assist_heartbeat",
        "assist_observer",
        "assist_settle_config",
        "assist_start",
        "assist_status",
        "assist_stop",
        "get_telemetry_fast",
        "set_absolute_angle",
        "set_angle",
        "set_current",
        "set_finger_angles",
        "set_gesture",
        "set_gesture_angle",
        "set_joint_model",
        "set_velocity",
        "stop",
    }
)


# Commands that remain serialized text on the primary CDC in a Protobuf build.
# This is an explicit list, not a catch-all, so a new firmware parser branch
# must make a conscious routing choice.
PROTOBUF_PRIMARY_TEXT_COMMANDS = frozenset(
    {
        "calibrate_exo",
        "calibrate_rom",
        "calibrate_rom_gesture",
        "cancel_rom",
        "check_limits",
        "clear_joint_trigger",
        "current_status",
        "cycle_gesture",
        "cycle_gesture_state",
        "debug",
        "disable",
        "disable_ids",
        "enable",
        "enable_ids",
        "gesture_list",
        "get_absolute_angle",
        "get_angle",
        "get_baud",
        "get_command_timeout",
        "get_current",
        "get_current_lim",
        "get_enabled",
        "get_exo_mode",
        "get_flip",
        "get_gesture",
        "get_gesture_angle",
        "get_gesture_angles",
        "get_gesture_sang",
        "get_gesture_state",
        "get_goal_acceleration",
        "get_goal_current",
        "get_goal_velocity",
        "get_hold_current",
        "get_home",
        "get_imu",
        "get_joint_model",
        "get_motor_limits",
        "get_motor_mode",
        "get_reply_route",
        "get_torque",
        "get_total_current_lim",
        "get_utc_time",
        "get_velocity",
        "help",
        "hold_position",
        "home",
        "info",
        "info_verbose",
        "io_profile",
        "led",
        "loop_stats",
        "oled",
        "reboot",
        "release_hold",
        "reset_io_profile",
        "reset_loop_stats",
        "set_baud",
        "set_command_timeout",
        "set_control_mode",
        "set_current_governor",
        "set_current_lim",
        "set_estimate_holdoff",
        "set_exo_mode",
        "set_flip",
        "set_gesture_state",
        "set_goal_acceleration",
        "set_goal_velocity",
        "set_hold_current",
        "set_home",
        "set_joint_trigger",
        "set_lower_limit",
        "set_motor_limits",
        "set_motor_mode",
        "set_reply_route",
        "set_total_current_lim",
        "set_upper_limit",
        "set_utc_time",
        "set_yaw_angle",
        "set_zero_offset",
        "shadow_config",
        "shadow_start",
        "shadow_status",
        "shadow_stop",
        "telemetry_diag",
        "version",
    }
)


# Typed-only batch operations have no parseMessage() equivalent.
PROTOBUF_ONLY_COMMANDS = frozenset(
    {"set_angles", "set_absolute_angles", "set_currents", "set_current_limits"}
)


def protobuf_command_route(command: str) -> str:
    """Return the declared Protobuf-build route for a known command."""
    normalized = str(command).strip().lower()
    if normalized in PROTOBUF_RPC_COMMANDS or normalized in PROTOBUF_ONLY_COMMANDS:
        return "protobuf-rpc"
    if normalized in PROTOBUF_PRIMARY_TEXT_COMMANDS:
        return "primary-text"
    raise KeyError(f"Unclassified firmware command: {normalized or command!r}")
