"""Tests for the post-flash read-only AMBAR runtime profile verifier."""

from __future__ import annotations

from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "usb_protocol"))

import probe_usb as probe  # noqa: E402
import rocket_protocol as protocol  # noqa: E402
import verify_firmware_profile as verifier  # noqa: E402


class FirmwareProfileVerifierTests(unittest.TestCase):
    class FakeClock:
        def __init__(self) -> None:
            self.now = 0.0

        def __call__(self) -> float:
            self.now += 0.01
            return self.now

    class FakePort:
        def __init__(self, response: bytes) -> None:
            self.response = response
            self.writes: list[bytes] = []
            self.reset_count = 0

        def reset_input_buffer(self) -> None:
            self.reset_count += 1

        def write(self, data: bytes) -> int:
            self.writes.append(bytes(data))
            return len(data)

        def read(self, size: int) -> bytes:
            chunk, self.response = self.response[:size], self.response[size:]
            return chunk

    @staticmethod
    def _actuator_payload(*, continuous_hil: bool, variable_hil: bool = False) -> bytes:
        reserved = (
            protocol.ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
            if continuous_hil
            else 0
        )
        if variable_hil:
            reserved |= protocol.ACTUATOR_STATUS_VARIABLE_HIL_PROFILE
        return struct.pack("<IIiiIBBH", 0, 0, 0, 0, 0, 0, 0, reserved)

    @staticmethod
    def _probe_response(
        *, features: int, continuous_hil: bool, variable_hil: bool = False
    ) -> bytes:
        ping_ack = protocol.encode_frame(
            protocol.PKT_ACK,
            100,
            1,
            struct.pack(
                "<HBBH",
                100,
                protocol.CMD_PING,
                protocol.ACK_OK,
                0,
            ),
        )
        snapshot_ack = protocol.encode_frame(
            protocol.PKT_ACK,
            101,
            2,
            struct.pack(
                "<HBBH",
                101,
                protocol.CMD_REQUEST_SNAPSHOT,
                protocol.ACK_OK,
                0,
            ),
        )
        heartbeat = protocol.encode_frame(
            protocol.PKT_HEARTBEAT,
            102,
            3,
            struct.pack("<IHH", features, 0, 0),
        )
        actuator = protocol.encode_frame(
            protocol.PKT_ACTUATOR_STATUS,
            103,
            4,
            FirmwareProfileVerifierTests._actuator_payload(
                continuous_hil=continuous_hil,
                variable_hil=variable_hil,
            ),
        )
        return ping_ack + snapshot_ack + heartbeat + actuator

    def test_safe_probe_sends_only_ping_and_snapshot_and_requires_fresh_status(self) -> None:
        features = (
            verifier.FEATURE_USB_PROTOCOL
            | verifier.FEATURE_SIMULATION
            | verifier.FEATURE_ACTUATOR
            | verifier.FEATURE_CONTINUOUS_HIL
        )
        port = self.FakePort(
            self._probe_response(features=features, continuous_hil=True)
        )
        result = probe.collect_read_only_status(
            port,
            seconds=1.0,
            sequence_start=100,
            clock=self.FakeClock(),
        )

        commands = []
        decoder = protocol.StreamDecoder()
        for write in port.writes:
            packets = decoder.feed(write)
            self.assertEqual(len(packets), 1)
            self.assertEqual(packets[0].packet_type, protocol.PKT_COMMAND)
            commands.append(packets[0].payload[0])

        self.assertEqual(
            commands,
            [protocol.CMD_PING, protocol.CMD_REQUEST_SNAPSHOT],
        )
        self.assertEqual(port.reset_count, 1)
        self.assertEqual(result.heartbeat["feature_flags"], features)
        self.assertTrue(result.actuator["continuous_hil_profile"])

    def test_safe_probe_rejects_missing_actuator_status(self) -> None:
        response = self._probe_response(features=0, continuous_hil=False)
        decoder = protocol.StreamDecoder()
        packets = decoder.feed(response)
        response_without_actuator = b"".join(
            protocol.encode_frame(
                packet.packet_type,
                packet.sequence,
                packet.time_ms,
                packet.payload,
            )
            for packet in packets
            if packet.packet_type != protocol.PKT_ACTUATOR_STATUS
        )
        port = self.FakePort(response_without_actuator)

        with self.assertRaisesRegex(
            probe.ReadOnlyProbeError,
            "fresh actuator status",
        ):
            probe.collect_read_only_status(
                port,
                seconds=0.1,
                sequence_start=100,
                clock=self.FakeClock(),
            )

    def test_auto_detection_rejects_multiple_ambar_boards(self) -> None:
        ports = [
            SimpleNamespace(
                device="COM4",
                description="AMBAR Airbrake USB",
                product="AMBAR",
                vid=0x0483,
                pid=0x5740,
            ),
            SimpleNamespace(
                device="COM7",
                description="AMBAR Airbrake USB",
                product="AMBAR",
                vid=0x0483,
                pid=0x5740,
            ),
        ]
        with self.assertRaisesRegex(SystemExit, "multiple AMBAR"):
            probe.choose_port(ports, None)

        class ListPorts:
            @staticmethod
            def comports():
                return ports

        with self.assertRaisesRegex(
            verifier.ProfileVerificationError,
            "pass --port",
        ):
            verifier._available_port(ListPorts, None)

    def test_normal_accepts_clear_simulation_and_hil_identity(self) -> None:
        summary = verifier.validate_runtime_profile(
            "Normal",
            {"feature_flags": verifier.FEATURE_USB_PROTOCOL | verifier.FEATURE_ACTUATOR},
            {"continuous_hil_profile": False},
        )
        self.assertEqual(summary["profile"], "Normal")

    def test_normal_rejects_each_hil_identity_channel(self) -> None:
        for features, actuator_profile in (
            (verifier.FEATURE_SIMULATION, False),
            (verifier.FEATURE_CONTINUOUS_HIL, False),
            (0, True),
        ):
            with self.subTest(
                features=features,
                actuator_profile=actuator_profile,
            ):
                with self.assertRaises(verifier.ProfileVerificationError):
                    verifier.validate_runtime_profile(
                        "Normal",
                        {"feature_flags": features},
                        {"continuous_hil_profile": actuator_profile},
                    )

    def test_continuous_hil_requires_all_feature_and_actuator_profile_bits(self) -> None:
        required = (
            verifier.FEATURE_USB_PROTOCOL
            | verifier.FEATURE_SIMULATION
            | verifier.FEATURE_ACTUATOR
            | verifier.FEATURE_CONTINUOUS_HIL
        )
        summary = verifier.validate_runtime_profile(
            "ContinuousHil",
            {"feature_flags": required},
            {"continuous_hil_profile": True},
        )
        self.assertEqual(summary["feature_flags"], required)

        for missing in (
            verifier.FEATURE_USB_PROTOCOL,
            verifier.FEATURE_SIMULATION,
            verifier.FEATURE_ACTUATOR,
            verifier.FEATURE_CONTINUOUS_HIL,
        ):
            with self.subTest(missing=missing):
                with self.assertRaises(verifier.ProfileVerificationError):
                    verifier.validate_runtime_profile(
                        "ContinuousHil",
                        {"feature_flags": required & ~missing},
                        {"continuous_hil_profile": True},
                    )

        with self.assertRaisesRegex(
            verifier.ProfileVerificationError,
            "actuator status",
        ):
            verifier.validate_runtime_profile(
                "ContinuousHil",
                {"feature_flags": required},
                {"continuous_hil_profile": False},
            )

    def test_variable_hil_requires_unique_identity_and_rejects_forced_profile(self) -> None:
        required = (
            verifier.FEATURE_USB_PROTOCOL
            | verifier.FEATURE_SIMULATION
            | verifier.FEATURE_ACTUATOR
            | verifier.FEATURE_VARIABLE_HIL
        )
        summary = verifier.validate_runtime_profile(
            "VariableHil",
            {"feature_flags": required},
            {"continuous_hil_profile": False, "variable_hil_profile": True},
        )
        self.assertTrue(summary["variable_hil_profile"])
        self.assertFalse(summary["continuous_hil_profile"])

        for heartbeat, actuator in (
            ({"feature_flags": required & ~verifier.FEATURE_VARIABLE_HIL},
             {"continuous_hil_profile": False, "variable_hil_profile": True}),
            ({"feature_flags": required},
             {"continuous_hil_profile": False, "variable_hil_profile": False}),
            ({"feature_flags": required | verifier.FEATURE_CONTINUOUS_HIL},
             {"continuous_hil_profile": False, "variable_hil_profile": True}),
            ({"feature_flags": required},
             {"continuous_hil_profile": True, "variable_hil_profile": True}),
        ):
            with self.subTest(heartbeat=heartbeat, actuator=actuator):
                with self.assertRaises(verifier.ProfileVerificationError):
                    verifier.validate_runtime_profile(
                        "VariableHil", heartbeat, actuator
                    )


if __name__ == "__main__":
    unittest.main()
