#include "i2c_interface.h"
#include "board_pins.h"
#include "driver/i2c.h"

#define I2C_PORT        I2C_NUM_0
#define I2C_FREQ_HZ     100000      /* TODO: SLF3S supports 400 kHz;
                                       100 kHz is the safe default for
                                       a first bring-up on a long cable */
#define I2C_TIMEOUT_MS  50


void i2c_interface_init(void)
{
    const i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_I2C_SDA,
        .scl_io_num       = PIN_I2C_SCL,
        /* External pull-ups are expected on the board; internal ones
           are weak (~45k) and marginal at 100 kHz with any cable
           capacitance. Enabled anyway so a missing resistor shows up as
           a marginal bus rather than a dead one. */
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
#if 0   /* TODO: enable with board_pins.h */
    i2c_param_config(I2C_PORT, &cfg);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
#else
    (void)cfg;
#endif
}


bool i2c_write(uint8_t addr7, const uint8_t *data, size_t len)
{
#if 0
    return i2c_master_write_to_device(I2C_PORT, addr7, data, len,
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK;
#else
    (void)addr7; (void)data; (void)len; return false;
#endif
}


bool i2c_read(uint8_t addr7, uint8_t *data, size_t len)
{
#if 0
    return i2c_master_read_from_device(I2C_PORT, addr7, data, len,
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK;
#else
    (void)addr7; (void)data; (void)len; return false;
#endif
}


/* Sensirion CRC-8: polynomial 0x31, init 0xFF, no reflection, no final
   XOR. Unrelated to the CRC-16 used for framing - different polynomial,
   different width, different job.
   VERIFY against the SLF3S datasheet before trusting it. */
static uint8_t sensirion_crc8(const uint8_t *d, size_t n)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u)
                                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}


bool i2c_read_sensirion(uint8_t addr7, uint16_t *words, size_t nwords)
{
    uint8_t raw[16];                       /* 3 bytes per word */
    const size_t n = nwords * 3u;
    if (n > sizeof(raw)) { return false; }

    if (!i2c_read(addr7, raw, n)) { return false; }

    for (size_t i = 0; i < nwords; i++) {
        const uint8_t *w = &raw[i * 3u];
        if (sensirion_crc8(w, 2) != w[2]) {
            return false;                  /* bit 3 fault condition */
        }
        words[i] = ((uint16_t)w[0] << 8) | w[1];
    }
    return true;
}