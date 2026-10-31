#include "pt100_driver.h"
#include "board_pins.h"
#include "driver/spi_master.h"
#include <math.h>

/* --- MAX31865 register map -------------------------------------------
 * VERIFY every constant against the MAX31865 datasheet (Table 1,
 * "Memory Map") before trusting a single reading. A wrong address here
 * produces a plausible number, not an error. */
#define MAX_REG_CONFIG      0x00
#define MAX_REG_RTD_MSB     0x01
#define MAX_REG_RTD_LSB     0x02
#define MAX_REG_FAULT_STAT  0x07
#define MAX_WRITE_FLAG      0x80   /* set bit 7 of the address to write */

/* Config byte. VERIFY against datasheet Table 2.
 *   bit7 VBIAS on
 *   bit6 conversion mode: 1 = automatic
 *   bit4 3-wire: SET THIS from the actual RTD wiring - 2/4-wire and
 *        3-wire need different values and the wrong one gives a
 *        consistent offset that looks like a calibration error
 *   bit0 filter: 1 = 50 Hz (Europe) */
#define MAX_CFG_VALUE   0xC1   /* TODO confirm the 3-wire bit */

/* Reference resistor fitted on the board. VERIFY against the BOM: 430R
   is standard for PT100, 4300R for PT1000. Getting this wrong scales
   every reading. */
#define RTD_RREF_OHMS   430.0f
#define RTD_R0_OHMS     100.0f

/* Callendar-Van Dusen coefficients, IEC 60751. */
#define CVD_A   3.9083e-3f
#define CVD_B  -5.775e-7f

static spi_device_handle_t dev;


static bool max_read_regs(uint8_t addr, uint8_t *buf, size_t len)
{
    spi_transaction_t t = {
        .flags     = 0,
        .cmd       = 0,
        .addr      = addr,
        .length    = len * 8,
        .rxlength  = len * 8,
        .rx_buffer = buf,
    };
    return spi_device_polling_transmit(dev, &t) == ESP_OK;
}


bool pt100_init(void)
{
    /* TODO: spi_bus_add_device() with PIN_CS_MAX31865.
       MAX31865 is SPI mode 1 (CPOL=0, CPHA=1) - VERIFY, the datasheet
       also allows mode 3, and the wrong mode reads shifted garbage.
       Then write MAX_CFG_VALUE to MAX_REG_CONFIG | MAX_WRITE_FLAG and
       allow one conversion period before the first read. */
    return false;
}


bool pt100_read(int16_t *temp_raw)
{
    uint8_t rx[2];
    if (!max_read_regs(MAX_REG_RTD_MSB, rx, sizeof(rx))) {
        return false;
    }

    const uint16_t word = ((uint16_t)rx[0] << 8) | rx[1];

    /* Bit 0 of the LSB is the fault flag, not data. Check it BEFORE
       using the value: a faulted conversion still returns a number. */
    if (word & 0x0001u) {
        return false;
    }

    const uint16_t adc = word >> 1;            /* 15-bit result */
    const float rrtd = ((float)adc * RTD_RREF_OHMS) / 32768.0f;

    /* Positive branch of the CVD equation. Below 0 degC a third-order
       term applies and this under-reads.
       TODO: decide whether sub-zero accuracy matters. The sample band is
       18-25 degC, but the vessel sees -30 degC on the ground per SED
       6.1, and health_monitor range-checks down to -40. */
    const float ratio = rrtd / RTD_R0_OHMS;
    const float disc  = CVD_A * CVD_A - 4.0f * CVD_B * (1.0f - ratio);
    if (disc < 0.0f) {
        return false;                           /* unphysical reading */
    }
    const float t_c = (-CVD_A + sqrtf(disc)) / (2.0f * CVD_B);

    /* Scale to 0.01 degC/LSB and clamp into int16 before casting -
       an out-of-range float cast to int16_t is undefined behaviour,
       not a wrap. */
    const float scaled = t_c * 100.0f;
    if (scaled < -32768.0f || scaled > 32767.0f) {
        return false;
    }
    *temp_raw = (int16_t)scaled;
    return true;
}