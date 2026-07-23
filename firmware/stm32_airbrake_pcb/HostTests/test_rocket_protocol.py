"""Python-side regression tests for the versioned AMBAR USB wire format.

The suite protects CRC/COBS framing, arbitrary serial chunking, signed fields,
and typed payload decoders shared with the C golden vectors.  It tests the
[ARCH-3]/[ARCH-6] codec only; no serial port or board is opened.
"""

from __future__ import annotations

from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "usb_protocol"))

import rocket_protocol as rp  # noqa: E402


class RocketProtocolTests(unittest.TestCase):
    def test_crc_check_vector(self) -> None:
        self.assertEqual(rp.crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_cross_language_golden_frame(self) -> None:
        frame = rp.encode_frame(
            rp.PKT_COMMAND,
            0x1234,
            0x89ABCDEF,
            bytes((rp.CMD_SET_TARGET_APOGEE, 2, 0xE8, 0x0B)),
        )
        self.assertEqual(frame.hex(), "10a502103412efcdab891002e80b9ff600")

    def test_arbitrary_chunk_splitting(self) -> None:
        frame = rp.command_frame(7, rp.CMD_PING, time_ms=123)
        decoder = rp.StreamDecoder()
        packets = []
        for chunk in (frame[:1], frame[1:2], frame[2:8], frame[8:-1], frame[-1:]):
            packets.extend(decoder.feed(chunk))
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].sequence, 7)

    def test_coalesced_frames(self) -> None:
        stream = (
            rp.command_frame(0xFFFF, rp.CMD_PING, time_ms=1)
            + rp.command_frame(0, rp.CMD_REQUEST_SNAPSHOT, time_ms=2)
        )
        packets = rp.StreamDecoder().feed(stream)
        self.assertEqual([packet.sequence for packet in packets], [0xFFFF, 0])

    def test_corrupt_and_oversized_frames_resynchronize(self) -> None:
        good = rp.command_frame(11, rp.CMD_PING, time_ms=3)
        corrupt = bytearray(good)
        corrupt[4] ^= 0x40
        oversized = bytes((1,)) * rp.MAX_FRAME + b"\x00"
        decoder = rp.StreamDecoder()
        packets = decoder.feed(bytes(corrupt) + oversized + good)
        self.assertEqual([packet.sequence for packet in packets], [11])
        self.assertGreaterEqual(decoder.errors, 2)

    def test_simulation_payload_round_trip(self) -> None:
        frame = rp.simulation_frame(
            9,
            altitude_m=123.456,
            acceleration_mps2=-9.81,
            velocity_mps=42.0,
            time_ms=4,
        )
        packet = rp.StreamDecoder().feed(frame)[0]
        self.assertEqual(packet.packet_type, rp.PKT_SIMULATION)
        self.assertEqual(len(packet.payload), 16)

    def test_event_and_heartbeat_decoders(self) -> None:
        event = rp.decode_event(struct.pack("<HHBBBBH", 0x10, 0x30, 1, 2, 3, 4, 5))
        self.assertEqual(event["changed_flags"], 0x10)
        self.assertEqual(event["current_state"], 2)
        self.assertEqual(event["detail"], 5)

        heartbeat = rp.decode_heartbeat(struct.pack("<IHH", 0x12345678, 9, 10))
        self.assertEqual(heartbeat["feature_flags"], 0x12345678)
        self.assertEqual(heartbeat["receive_errors"], 9)
        self.assertEqual(heartbeat["transmit_drops"], 10)

    def test_hil_override_frame_has_one_validated_mode_byte(self) -> None:
        frame = rp.hil_override_frame(
            22,
            rp.HIL_OVERRIDE_FORCE_FULL,
            time_ms=99,
        )
        packet = rp.StreamDecoder().feed(frame)[0]
        self.assertEqual(packet.packet_type, rp.PKT_COMMAND)
        self.assertEqual(
            packet.payload,
            bytes(
                (
                    rp.CMD_HIL_SET_OVERRIDE,
                    1,
                    rp.HIL_OVERRIDE_FORCE_FULL,
                )
            ),
        )
        with self.assertRaises(rp.ProtocolError):
            rp.hil_override_frame(23, 3, time_ms=100)

    def test_known_full_recovery_frame_has_exact_command_and_magic(self) -> None:
        self.assertEqual(rp.CMD_RECOVER_KNOWN_FULL_RETRACT, 0x25)
        self.assertEqual(rp.RECOVER_KNOWN_FULL_MAGIC, b"FULL")
        frame = rp.recover_known_full_retract_frame(24, time_ms=101)
        packet = rp.StreamDecoder().feed(frame)[0]
        self.assertEqual(packet.packet_type, rp.PKT_COMMAND)
        self.assertEqual(
            packet.payload,
            bytes((rp.CMD_RECOVER_KNOWN_FULL_RETRACT, 4)) + b"FULL",
        )

    def test_actuator_reserved_bits_decode_without_payload_growth(self) -> None:
        reserved = (
            rp.ACTUATOR_STATUS_HOME_ACTIVE
            | rp.ACTUATOR_STATUS_LIMITS_PLAUSIBLE
            | rp.ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE
            | (
                rp.HIL_OVERRIDE_FORCE_HOME
                << rp.ACTUATOR_STATUS_HIL_OVERRIDE_SHIFT
            )
            | rp.ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
            | rp.ACTUATOR_STATUS_ENDPOINT_SEQUENCE_VERIFIED
        )
        payload = struct.pack(
            "<IIiiIBBH",
            1,
            2,
            0,
            -4,
            0,
            3,
            0x4D,
            reserved,
        )
        self.assertEqual(len(payload), 24)
        status = rp.decode_actuator_status(payload)
        self.assertTrue(status["software_home_active"])
        self.assertFalse(status["software_full_active"])
        self.assertTrue(status["geometry_plausible"])
        self.assertTrue(status["stroke_sequence_verified"])
        # Protocol-v2 compatibility keys remain present.
        self.assertTrue(status["home_active"])
        self.assertFalse(status["full_active"])
        self.assertTrue(status["limits_plausible"])
        self.assertTrue(status["hil_override_active"])
        self.assertEqual(
            status["hil_override_mode"],
            rp.HIL_OVERRIDE_FORCE_HOME,
        )
        self.assertTrue(status["continuous_hil_profile"])
        self.assertFalse(status["variable_hil_profile"])
        self.assertTrue(status["endpoint_sequence_verified"])

    def test_variable_hil_state_correlates_sequence_and_separates_fractions(self) -> None:
        flags = (
            rp.VARIABLE_HIL_FLAG_DRIVER_OK
            | rp.VARIABLE_HIL_FLAG_CONFIG_VALID
            | rp.VARIABLE_HIL_FLAG_SIM_ACTIVE
            | rp.VARIABLE_HIL_FLAG_SIM_FRESH
            | rp.VARIABLE_HIL_FLAG_ARMED
            | rp.VARIABLE_HIL_FLAG_SOFTWARE_HOME
            | rp.VARIABLE_HIL_FLAG_TARGET_REACHABLE
        )
        payload = struct.pack(
            "<HHHHiiIIIIiiBBBB",
            0xBEEF,
            32768,
            30000,
            28000,
            70300,
            65536,
            0x11223344,
            0x55667788,
            0xAABBCCDD,
            0x12345678,
            31500,
            28500,
            3,
            4,
            flags,
            rp.VARIABLE_HIL_FEEDBACK_TMC5240_XACTUAL,
        )
        state = rp.decode_variable_hil_state(payload)
        self.assertEqual(state["simulation_sequence"], 0xBEEF)
        self.assertAlmostEqual(state["controller_requested_fraction"], 32768 / 65535)
        self.assertAlmostEqual(state["actuator_target_fraction"], 30000 / 65535)
        self.assertAlmostEqual(state["xactual_fraction"], 28000 / 65535)
        self.assertEqual(state["target_steps"], 70300)
        self.assertEqual(state["actual_steps"], 65536)
        self.assertTrue(state["target_reachable"])
        self.assertAlmostEqual(state["closed_predicted_apogee_m"], 3150.0)
        self.assertAlmostEqual(state["full_predicted_apogee_m"], 2850.0)
        self.assertFalse(state["driver_enabled"])
        self.assertEqual(
            state["feedback_source"],
            rp.VARIABLE_HIL_FEEDBACK_TMC5240_XACTUAL,
        )

    def test_variable_hil_config_upload_is_atomic_versioned_and_crc_checked(self) -> None:
        config = rp.VariableHilConfig(
            calibration_version=5,
            control_mode=1,
            predictor_mode=1,
            target_apogee_m=914.4,
            mission_tolerance_m=30.48,
            control_deadband_m=3.048,
            full_deployment_error_m=76.2,
            minimum_deploy_altitude_m=100.0,
            minimum_flight_time_s=1.5,
            predictive_update_period_s=0.05,
            coast_mass_kg=5.0,
            maximum_deploy_fraction=1.0,
            deployment_hysteresis_fraction=0.02,
            deployment_cda_m2=(0.012, 0.014, 0.017, 0.021, 0.026),
            sea_level_air_density_kgpm3=1.225,
            density_scale_height_m=8500.0,
            launch_site_elevation_m=250.0,
            actuator_delay_s=0.10,
            actuator_open_rate_fraction_per_s=0.864,
            actuator_close_rate_fraction_per_s=0.844,
        )
        payload = rp.encode_variable_hil_config_payload(config)
        self.assertEqual(len(payload), rp.VARIABLE_HIL_CONFIG_PAYLOAD_SIZE)
        decoded = rp.decode_variable_hil_config(payload)
        self.assertEqual(decoded["calibration_version"], 5)
        self.assertAlmostEqual(decoded["target_apogee_m"], 914.4)
        self.assertEqual(len(decoded["deployment_cda_m2"]), 5)
        self.assertAlmostEqual(decoded["actuator_open_rate_fraction_per_s"], 0.864)

        frame = rp.variable_hil_config_upload_frame(77, config, time_ms=123)
        packet = rp.StreamDecoder().feed(frame)[0]
        self.assertEqual(packet.packet_type, rp.PKT_VARIABLE_HIL_CONFIG_UPLOAD)
        self.assertEqual(packet.sequence, 77)
        self.assertEqual(packet.payload, payload)

        corrupt = bytearray(payload)
        corrupt[10] ^= 1
        with self.assertRaisesRegex(rp.ProtocolError, "CRC mismatch"):
            rp.decode_variable_hil_config(bytes(corrupt))


if __name__ == "__main__":
    unittest.main()
