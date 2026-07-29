"""Frame construction and parsing for the ESP32 <-> Pi UART link.

See docs/interfaces/uart_esp32_pi.md for the frame format.
"""


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE.

    Polynomial 0x1021, init 0xFFFF, no reflection, no final XOR.
    Computed over message ID + length + payload (not the sync bytes).
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8            # bring the byte into the top 8 bits
        for _ in range(8):          # process one bit at a time
            if crc & 0x8000:        # is bit 15 set?
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF           # Python ints are unbounded - must truncate
    return crc