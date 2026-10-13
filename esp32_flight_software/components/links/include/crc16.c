#include "crc16.h"

/* CRC-16/CCITT-FALSE.
 *
 * These parameters are fixed by the protocol and must never be changed on
 * one side alone. raspberry_pi_software/src/stard/links/framing.py
 * implements the same algorithm; the two must agree byte for byte.
 *
 *   polynomial      0x1021   (x^16 + x^12 + x^5 + 1)
 *   initial value   0xFFFF   - not 0x0000. With a zero init an all-zero
 *                              message produces a zero CRC, so a dead
 *                              transmitter would pass its own check.
 *   input reflect   no
 *   output reflect  no
 *   final XOR       none
 *
 * Check value: crc16_ccitt("123456789", 9) == 0x29B1
 *
 * crc is uint16_t deliberately. The arithmetic promotes to int, so
 * (crc << 1) produces a 17-bit intermediate; the bit above position 15 is
 * discarded by the conversion back to uint16_t on assignment. That
 * truncation is part of the algorithm, not an accident - widening the
 * accumulator is not a safe "optimisation".
 */
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

bool crc16_self_test(void)
{
    static const uint8_t check[] = "123456789";
    /* sizeof(check) is 10 - the string literal carries a terminating NUL.
     * The standard check value is defined over the nine digits only. */
    return crc16_ccitt(check, sizeof(check) - 1) == 0x29B1;
}