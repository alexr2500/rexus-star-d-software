from stard.links.framing import crc16_ccitt


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