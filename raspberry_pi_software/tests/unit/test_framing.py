from stard.links.framing import crc16_ccitt
from stard.links.framing import build_frame
from stard.links import protocol
from stard.links.framing import FrameParser
import pytest


def test_crc_standard_check_value():
    """The published check value for CRC-16/CCITT-FALSE."""
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_crc_empty_input_returns_init():
    assert crc16_ccitt(b"") == 0xFFFF


def test_crc_detects_single_bit_flip():
    original = bytes([0x01, 0x06, 0x02, 0x00])
    corrupted = bytes([0x01, 0x06, 0x02, 0x01])
    assert crc16_ccitt(original) != crc16_ccitt(corrupted)


def test_crc_detects_byte_swap():
    """A plain checksum would miss this; a CRC must not."""
    assert crc16_ccitt(bytes([0x12, 0x34])) != crc16_ccitt(bytes([0x34, 0x12]))


def test_build_frame_rejects_oversized_payload():
    with pytest.raises(ValueError):                                         # error must be raised for test to pass
        build_frame(0x01, b"\x00" * (protocol.PROTO_MAX_PAYLOAD + 1))       # b"\x00" * n makes n 0 bytes


def test_build_frame_length():
    frame = build_frame(protocol.MessageId.MSG_POLL, b"\x01\x02\x03")
    assert len(frame) == 3 + protocol.PROTO_OVERHEAD_BYTES


def test_build_frame_header_fields():
    payload = b"\xAA\xBB"
    frame = build_frame(protocol.MessageId.MSG_STATUS, payload)
    assert frame[0] == protocol.PROTO_SYNC0
    assert frame[1] == protocol.PROTO_SYNC1
    assert frame[2] == protocol.MessageId.MSG_STATUS
    assert frame[3] == len(payload)


def test_build_frame_crc_is_little_endian():
    payload = b"\x01"
    frame = build_frame(0x01, payload)
    body = frame[2:-2]                      # ID + length + payload
    expected = crc16_ccitt(body)
    assert frame[-2] == expected & 0xFF          # low byte first
    assert frame[-1] == (expected >> 8) & 0xFF   # high byte second


def test_parser_single_frame():
    p = FrameParser()
    frame = build_frame(protocol.MessageId.MSG_POLL, b"\x01\x02\x03")
    assert p.feed(frame) == [(protocol.MessageId.MSG_POLL, b"\x01\x02\x03")]


def test_parser_split_across_feeds():
    p = FrameParser()
    frame = build_frame(0x01, b"\x01\x02\x03\x04\x05\x06")
    assert p.feed(frame[:3]) == []
    assert p.feed(frame[3:7]) == []
    assert p.feed(frame[7:]) == [(0x01, b"\x01\x02\x03\x04\x05\x06")]


def test_parser_skips_leading_garbage():
    p = FrameParser()
    frame = build_frame(0x03, b"\xDE\xAD")
    assert p.feed(b"\x11\x22\x33" + frame) == [(0x03, b"\xDE\xAD")]


def test_parser_two_frames_in_one_feed():
    p = FrameParser()
    a = build_frame(0x01, b"\x01")
    b = build_frame(0x03, b"\x02")
    assert p.feed(a + b) == [(0x01, b"\x01"), (0x03, b"\x02")]


def test_parser_rejects_corrupted_payload():
    p = FrameParser()
    frame = bytearray(build_frame(0x01, b"\x01\x02\x03"))
    frame[5] ^= 0xFF                    # flip every bit of one payload byte
    assert p.feed(bytes(frame)) == []


def test_parser_survives_corrupted_length():
    """A bad length byte must not hang the parser or swallow later frames."""
    p = FrameParser()
    bad = bytearray(build_frame(0x01, b"\x01\x02"))
    bad[3] = 0xFF                       # claim a 255-byte payload
    good = build_frame(0x03, b"\x09")
    assert p.feed(bytes(bad) + good) == [(0x03, b"\x09")]


def test_parser_sync_split_across_feeds():
    """0xAA ends one chunk, 0x55 starts the next."""
    p = FrameParser()
    frame = build_frame(0x01, b"\x07")
    assert p.feed(frame[:1]) == []      # just the 0xAA
    assert p.feed(frame[1:]) == [(0x01, b"\x07")]