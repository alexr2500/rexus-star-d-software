#include "pt100_driver.h"
#include "board_pins.h"
#include "spi_interface.h"
#include <math.h>
#include <stddef.h>

/* ---- MAX31865 register map ------------------------------------------- */
#define MAX_REG_CONFIG        0x00
#define MAX_REG_RTD_MSB       0x01
#define MAX_REG_FAULT_STATUS  0x07
#define MAX_WRITE_BIT         0x80   /* set on the address byte to write */

/* Config: VBIAS on, auto conversion, 2/4-wire, clear faults, 50 Hz filter.
   bit7 VBIAS=1, bit6 AUTO=1, bit5 1-shot=0, bit4 3WIRE=0,
   bit3:2 fault detect=00, bit1 fault clear=1, bit0 filter=1 (50 Hz). */
#define MAX_CONFIG_VALUE      0xC3

/* Reference resistor fitted on the board. CHECK THIS against the BOM
   before flight: the whole temperature scale is proportional to it. */
#define PT100_RREF_OHMS       430.0f
#define PT100_R0_OHMS         100.0f

/* Callendar-Van Dusen coefficients, IEC 60751. */
#define CVD_A                 3.9083e-3f
#define CVD_B                -5.775e-7f

/* Plausibility band. Anything outside is a wiring or sensor fault, not a
   temperature: the experiment lives at 18-25 C and ground ops at worst
   a few degrees either side of room temperature. */
#define PT100_MIN_CENTI_C    (-4000)   /* -40.00 C */
#define PT100_MAX_CENTI_C    ( 8500)   /*  85.00 C */

#define PT100_SPI_CLOCK_HZ    1000000  /* MAX31865 max is 5 MHz */
#define PT100_SPI_MODE        1        /* CPOL=0, CPHA=1 */

static spi_device_handle_t dev;
static uint8_t last_fault;

static esp_err_t reg_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg | MAX_WRITE_BIT), value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_polling_transmit(dev, &t);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t n)
{
    uint8_t tx[1 + 4] = { (uint8_t)(reg & 0x7Fu) };
    uint8_t rx[1 + 4] = { 0 };
    if (n > 4) return ESP_ERR_INVALID_ARG;

    spi_transaction_t t = {
        .length    = 8u * (n + 1u),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(dev, &t);
    if (err != ESP_OK) return err;

    for (size_t i = 0; i < n; i++) buf[i] = rx[i + 1];   /* skip the address echo */
    return ESP_OK;
}

esp_err_t pt100_driver_init(void)
{
    esp_err_t err = spi_interface_add_device(PIN_CS_MAX31865,
                                             PT100_SPI_CLOCK_HZ,
                                             PT100_SPI_MODE, &dev);
    if (err != ESP_OK) return err;

    return reg_write(MAX_REG_CONFIG, MAX_CONFIG_VALUE);
}

uint8_t pt100_driver_last_fault(void) { return last_fault; }

esp_err_t pt100_driver_read(int16_t *out_centi_c)
{
    if (out_centi_c == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t rtd[2];
    esp_err_t err = reg_read(MAX_REG_RTD_MSB, rtd, 2);
    if (err != ESP_OK) return err;

    const uint16_t raw = (uint16_t)((uint16_t)rtd[0] << 8) | rtd[1];

    /* Bit 0 of the RTD register is the fault flag, NOT data. Check it
       before using the number, or a broken sensor reads as a plausible
       temperature. */
    if (raw & 0x0001u) {
        uint8_t f = 0;
        (void)reg_read(MAX_REG_FAULT_STATUS, &f, 1);
        last_fault = f;
        (void)reg_write(MAX_REG_CONFIG, MAX_CONFIG_VALUE);  /* clears faults */
        return ESP_FAIL;
    }
    last_fault = 0;

    const uint16_t adc = (uint16_t)(raw >> 1);              /* 15-bit result */
    const float    r   = ((float)adc / 32768.0f) * PT100_RREF_OHMS;

    /* Positive branch of the Callendar-Van Dusen equation:
         R(T) = R0 (1 + A T + B T^2)   solved for T.
       Valid for T >= 0 C. Below 0 C it drifts, which is acceptable here:
       anything below 0 C is already outside the plausibility band. */
    const float disc = (CVD_A * CVD_A)
                     - 4.0f * CVD_B * (1.0f - r / PT100_R0_OHMS);
    if (disc < 0.0f) return ESP_FAIL;

    const float t_c = (-CVD_A + sqrtf(disc)) / (2.0f * CVD_B);

    /* Clamp BEFORE the cast. Casting a float outside int16 range is
       undefined behaviour, not a wrapped number. */
    const float centi = t_c * 100.0f;
    if (centi < (float)PT100_MIN_CENTI_C) return ESP_FAIL;
    if (centi > (float)PT100_MAX_CENTI_C) return ESP_FAIL;

    *out_centi_c = (int16_t)centi;
    return ESP_OK;
}