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


if __name__ == "__main__":
    unittest.main()
