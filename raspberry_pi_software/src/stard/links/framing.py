from stard.links import protocol

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


def build_frame(msg_id: int, payload: bytes) -> bytes:
    if len(payload) > protocol.PROTO_MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)}")

    body = bytes([msg_id, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([protocol.PROTO_SYNC0, protocol.PROTO_SYNC1]) + body + crc.to_bytes(2, "little")


class FrameParser:
    """Extracts complete, CRC-valid frames from an arbitrary byte stream."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf.extend(data)          # append new bytes to what we already had
        frames: list[tuple[int, bytes]] = []

        while True:
            # 1. Not enough bytes for even the smallest frame?
            if len(self._buf) < protocol.PROTO_OVERHEAD_BYTES:
                break

            # 2. Find the sync pattern
            sync = bytes([protocol.PROTO_SYNC0, protocol.PROTO_SYNC1])
            pos = self._buf.find(sync)
            if pos == -1:
                # No sync anywhere. Keep only the last byte in case
                # 0xAA is the start of a sync split across two feeds.
                del self._buf[:-1]
                break
            if pos > 0:
                del self._buf[:pos]     # discard garbage before the sync
                continue                # re-check length from the top

            # 3. Read the header
            msg_id = self._buf[2]
            length = self._buf[3]

            # 4. Implausible length: cannot be a real frame.
            #    Nudge forward one byte and resume hunting.
            if length > protocol.PROTO_MAX_PAYLOAD:
                del self._buf[:1]
                continue

            # 5. Is the whole frame here yet?
            total = 4 + length + 2
            if len(self._buf) < total:
                break                   # legitimate frame, still arriving

            # 6. Check the CRC over ID + length + payload
            body = bytes(self._buf[2 : 4 + length])
            received = int.from_bytes(self._buf[4 + length : total], "little")

            # 7. Emit on success, resynchronise on failure
            if crc16_ccitt(body) == received:
                payload = bytes(self._buf[4 : 4 + length])
                frames.append((msg_id, payload))
                del self._buf[:total]
            else:
                del self._buf[:1]
            continue

        return frames