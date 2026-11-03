#ifndef ABP_DRIVER_H
#define ABP_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Honeywell ABP series board-mount pressure sensor, off-board on J1:
     SCK  J1.3   MISO J1.4   CS J1.5   (no MOSI -- the part is read-only)
   Measures pressure in the microfluidic system (SED table 4.5).

   The ABP is read-only in the strict sense: there is no register map and
   no command to send. Asserting CS and clocking out four bytes IS the
   whole protocol. */

/* Status, from the top two bits of the first byte. */
typedef enum {
    ABP_STATUS_NORMAL     = 0,  /* good data                              */
    ABP_STATUS_CMD_MODE   = 1,  /* device in command mode: should never
                                   happen in flight, it means the part was
                                   put into factory mode                  */
    ABP_STATUS_STALE      = 2,  /* this sample was already fetched -- we
                                   polled faster than the sensor updates  */
    ABP_STATUS_DIAGNOSTIC = 3   /* the part has detected its own fault     */
} abp_status_t;

esp_err_t abp_driver_init(void);

/* One reading.
 *   *out_counts  raw 14-bit pressure, 0..16383
 *   *out_temp_centi_c  internal temperature in 0.01 C, or left alone on a
 *                      part without the temperature option
 * Either pointer may be NULL. Returns ESP_OK only when the status bits
 * say NORMAL; on anything else *out_* are untouched and the reason is
 * available from abp_driver_last_status().
 *
 * RAW COUNTS, deliberately. The counts-to-pascals transfer function needs
 * the part's pressure range, which is encoded in the part number and is
 * not yet confirmed (see ABP_RANGE_* below). Transmitting counts keeps
 * the flight data correct and range-independent: the conversion can be
 * applied on the ground, and corrected later without reflying. */
esp_err_t abp_driver_read(int16_t *out_counts, int16_t *out_temp_centi_c);

/* Status from the most recent read. */
abp_status_t abp_driver_last_status(void);

/* ---- transfer function ------------------------------------------------
 * Honeywell ABP output spans 10% to 90% of the 14-bit range:
 *     counts_min = 0.10 * 2^14 = 1638
 *     counts_max = 0.90 * 2^14 = 14746
 * so
 *     P = (counts - 1638) * (Pmax - Pmin) / (14746 - 1638) + Pmin
 *
 * TODO: ABP_RANGE_MIN_PA and ABP_RANGE_MAX_PA come from the part number
 * on the BOM (for example ABPDANT005PGAA5 is 0..5 psi gauge). The values
 * below are a placeholder and are used ONLY by the helper function, never
 * by the flight data path -- the telemetry carries raw counts. Fix them
 * when the part is confirmed, and record them in the ICD so the ground
 * station converts with the same numbers.
 */
#define ABP_COUNTS_MIN      1638
#define ABP_COUNTS_MAX      14746

#define ABP_RANGE_MIN_PA    0        /* TODO: confirm from the part number */
#define ABP_RANGE_MAX_PA    34474    /* TODO: placeholder = 5 psi gauge     */

/* Converts raw counts to pascals using the constants above. Not used in
   the flight data path; provided for the self-test and for bench work. */
int32_t abp_counts_to_pa(int16_t counts);

#endif /* ABP_DRIVER_H */