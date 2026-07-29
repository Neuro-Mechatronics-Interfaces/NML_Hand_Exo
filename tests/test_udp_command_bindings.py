import unittest

from nml_hand_exo.interface._udp_command_bindings import (
    DEFAULT_EASE_DURATION_MS,
    DEFAULT_PULSE_DURATION_MS,
    DEFAULT_PULSE_SHAPE,
    DEFAULT_PULSE_STEP_MS,
    UDP_BINDING_SCHEMA_VERSION,
    UDP_CONNECTION_PORT_THRESHOLD,
    UDP_HEARTBEAT_REQUEST_VALUE,
    binding_lookup,
    default_bindings,
    expand_command_templates,
    make_default_binding_profile,
    make_index_middle_pinch_profile,
    normalize_binding_profile,
    parse_udp_integer,
)


class UDPCommandBindingsTests(unittest.TestCase):
    def test_connection_port_protocol_starts_above_64(self):
        self.assertEqual(UDP_CONNECTION_PORT_THRESHOLD, 64)
        self.assertEqual(UDP_HEARTBEAT_REQUEST_VALUE, 1023)
        self.assertGreater(
            UDP_HEARTBEAT_REQUEST_VALUE, UDP_CONNECTION_PORT_THRESHOLD
        )

    def test_parses_plain_and_json_integer_payloads(self):
        self.assertEqual(parse_udp_integer("+99"), 99)
        self.assertEqual(parse_udp_integer(" -5\n"), -5)
        self.assertEqual(parse_udp_integer("99"), 99)
        self.assertEqual(parse_udp_integer("{\"value\": -99}"), -99)
        self.assertIsNone(parse_udp_integer("1.0"))
        self.assertIsNone(parse_udp_integer("true"))
        self.assertIsNone(parse_udp_integer('{"command":"version"}'))

    def test_default_torque_map_matches_signed_digit_contract(self):
        lookup = {row["value"]: row for row in default_bindings("torque")}
        self.assertEqual(set(lookup), set(range(-5, 6)))
        self.assertEqual(lookup[0]["command"], "stop:all")
        self.assertEqual(lookup[1]["command"], "set_current:{thumbflex}:100")
        self.assertEqual(lookup[-1]["command"], "set_current:{thumbflex}:-100")
        self.assertEqual(lookup[5]["command"], "set_current:{pinky}:100")
        self.assertEqual(lookup[-5]["command"], "set_current:{pinky}:-100")

    def test_position_defaults_map_each_digit_to_its_own_gesture(self):
        lookup = {row["value"]: row for row in default_bindings("position")}
        self.assertEqual(set(lookup), set(range(-5, 6)))
        self.assertEqual(lookup[0]["command"], "set_gesture:grasp:open")
        for value, digit, _motor in (
            (1, "thumb", None),
            (2, "index", None),
            (3, "middle", None),
            (4, "ring", None),
            (5, "pinky", None),
        ):
            self.assertEqual(lookup[value]["command"], f"set_gesture:{digit}:flex")
            self.assertEqual(lookup[-value]["command"], f"set_gesture:{digit}:extend")

    def test_profile_rejects_duplicate_and_connection_port_values(self):
        profile = make_default_binding_profile()
        profile["bindings"].append(dict(profile["bindings"][0]))
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            normalize_binding_profile(profile)

        profile = make_default_binding_profile()
        profile["bindings"][0]["value"] = UDP_CONNECTION_PORT_THRESHOLD + 1
        with self.assertRaisesRegex(ValueError, "connection-port"):
            normalize_binding_profile(profile)

        profile = make_default_binding_profile()
        profile["bindings"][0]["value"] = -(UDP_CONNECTION_PORT_THRESHOLD + 1)
        with self.assertRaisesRegex(ValueError, "connection-port"):
            normalize_binding_profile(profile)

    def test_lookup_normalizes_string_integer_values(self):
        profile = make_default_binding_profile()
        profile["bindings"][0]["value"] = "-5"
        lookup = binding_lookup(profile)
        self.assertIn(-5, lookup)

    def test_legacy_fixed_heartbeat_fields_are_migrated_away(self):
        profile = make_default_binding_profile()
        profile["schema_version"] = 1
        profile["heartbeat"] = {"live": 99, "disconnected": -99}
        normalized = normalize_binding_profile(profile)
        self.assertEqual(normalized["schema_version"], UDP_BINDING_SCHEMA_VERSION)
        self.assertEqual(UDP_BINDING_SCHEMA_VERSION, 3)
        self.assertNotIn("heartbeat", normalized)

    def test_index_middle_pinch_profile_is_posture_pinch_map(self):
        profile = make_index_middle_pinch_profile()
        normalized = normalize_binding_profile(profile)
        self.assertEqual(normalized["control_mode"], "position")
        lookup = binding_lookup(normalized)
        self.assertEqual(set(lookup), {0, 2, 3})
        self.assertEqual(lookup[2]["command"], "set_gesture:pinch_index:close")
        self.assertEqual(lookup[3]["command"], "set_gesture:pinch_middle:close")
        # REST opens both pinches so a "let go" packet fully releases the hand.
        self.assertIn("pinch_index:open", lookup[0]["command"])
        self.assertIn("pinch_middle:open", lookup[0]["command"])

    def test_pulse_fields_default_and_backward_compatible(self):
        # A legacy v2 profile with no pulse fields still loads with defaults.
        legacy = make_default_binding_profile()
        for key in (
            "pulse_shape",
            "pulse_duration_ms",
            "pulse_step_ms",
            "ease_duration_ms",
        ):
            legacy.pop(key, None)
        legacy["schema_version"] = 2
        normalized = normalize_binding_profile(legacy)
        self.assertEqual(normalized["pulse_shape"], DEFAULT_PULSE_SHAPE)
        self.assertEqual(
            normalized["pulse_duration_ms"], DEFAULT_PULSE_DURATION_MS
        )
        self.assertEqual(normalized["pulse_step_ms"], DEFAULT_PULSE_STEP_MS)
        self.assertEqual(
            normalized["ease_duration_ms"], DEFAULT_EASE_DURATION_MS
        )

    def test_pulse_fields_are_validated(self):
        profile = make_default_binding_profile()
        profile["pulse_step_ms"] = 0
        with self.assertRaisesRegex(ValueError, "pulse_step_ms"):
            normalize_binding_profile(profile)

        profile = make_default_binding_profile()
        profile["pulse_shape"] = "square"
        with self.assertRaisesRegex(ValueError, "pulse_shape"):
            normalize_binding_profile(profile)

    def test_expands_motor_placeholders_to_explicit_ids(self):
        targets = {"index": [6, 16], "middle": [7]}
        self.assertEqual(
            expand_command_templates("set_current:{index}:100", targets),
            ["set_current:6:100", "set_current:16:100"],
        )
        self.assertEqual(
            expand_command_templates(
                "set_current:{middle}:50\nstop:6", targets
            ),
            ["set_current:7:50", "stop:6"],
        )
        with self.assertRaisesRegex(ValueError, "No active motor"):
            expand_command_templates("set_current:{ring}:50", targets)


if __name__ == "__main__":
    unittest.main()
