#ifndef CRC16_H
#define CRC16_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* CRC-16/CCITT-FALSE.
 *
 *   polynomial      0x1021
 *   initial value   0xFFFF
 *   input reflect   no
 *   output reflect  no
 *   final XOR       none
 *
 * These parameters are fixed by the ESP32 <-> Pi protocol and by the TCU
 * downlink format. They are mirrored in
 * raspberry_pi_software/src/stard/links/framing.py and must not be changed
 * on one side alone.
 *
 * Standard check value: crc16_ccitt("123456789", 9) == 0x29B1
 */

/* Remainder over len bytes starting at data.
 * len == 0 returns the initial value, 0xFFFF. data must not be NULL
 * unless len is 0.
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/* Power-on self test: recomputes the standard check value and compares it
 * against 0x29B1. Returns true on match. Costs ~72 shift-XOR iterations.
 *
 * Called once from the startup sequence. A false return means the CRC
 * routine itself is wrong, which invalidates every frame and every
 * telemetry packet, so it is a boot-time abort condition rather than an
 * error flag.
 */
bool crc16_self_test(void);

#endif /* CRC16_H */