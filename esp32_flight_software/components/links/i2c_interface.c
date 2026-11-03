#include "i2c_interface.h"
#include "board_pins.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stddef.h>
#include <string.h>

#define I2C_PORT            I2C_NUM_0
#define I2C_CLOCK_HZ        100000     /* SLF3S supports 100 kHz; the sensor is
                                          off-board on a harness, so slow is right */
#define I2C_TIMEOUT_MS      50

#define SLF3S_ADDR          0x08
/* Start continuous measurement, water calibration. */
#define SLF3S_CMD_START_H   0x36
#define SLF3S_CMD_START_L   0x08
/* Stop continuous measurement. */
#define SLF3S_CMD_STOP_H    0x3F
#define SLF3S_CMD_STOP_L    0xF9

/* First measurement is available ~12 ms after the start command. */
#define SLF3S_START_DELAY_MS 15

/* Sensirion CRC-8: polynomial 0x31, initial value 0xFF, no reflection,
   no final XOR. Computed over each 2-byte word; the sensor appends one
   CRC byte per word, so a 3-word read is 9 bytes.
   This is NOT the same CRC as the ESP<->Pi link, which is CRC-16/CCITT. */
static uint8_t sensirion_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((uint8_t)(crc << 1) ^ 0x31u)
                                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

esp_err_t i2c_interface_init(void)
{
    i2c_config_t cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.mode             = I2C_MODE_MASTER;
    cfg.sda_io_num       = PIN_I2C_SDA;
    cfg.scl_io_num       = PIN_I2C_SCL;
    /* The board carries its own pull-ups (R_I2C_SCL1 / R_I2C_SCL2), which
       are the right strength for the harness length. The internal ones
       would sit in parallel and are far too weak to matter, but enabling
       them also holds GPIO3 -- a strapping pin -- high at reset, so they
       stay off deliberately. */
    cfg.sda_pullup_en    = 0;
    cfg.scl_pullup_en    = 0;
    cfg.master.clk_speed = I2C_CLOCK_HZ;

    esp_err_t err = i2c_param_config(I2C_PORT, &cfg);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return err;

    const uint8_t start[2] = { SLF3S_CMD_START_H, SLF3S_CMD_START_L };
    err = i2c_master_write_to_device(I2C_PORT, SLF3S_ADDR, start, sizeof start,
                                     pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(SLF3S_START_DELAY_MS));
    return ESP_OK;
}

esp_err_t i2c_interface_read_flow(int16_t *flow_raw,
                                  int16_t *temp_raw,
                                  uint16_t *flags)
{
    uint8_t b[9];
    esp_err_t err = i2c_master_read_from_device(I2C_PORT, SLF3S_ADDR, b, sizeof b,
                                                pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;

    /* Three words, each followed by its own CRC. Check all three before
       using any of them: a sensor that has lost power mid-harness returns
       0xFF everywhere, which decodes to a plausible-looking flow. */
    for (int w = 0; w < 3; w++) {
        if (sensirion_crc8(&b[w * 3], 2) != b[w * 3 + 2]) {
            return ESP_ERR_INVALID_CRC;
        }
    }

    if (flow_raw) *flow_raw = (int16_t)((uint16_t)b[0] << 8 | b[1]);
    if (temp_raw) *temp_raw = (int16_t)((uint16_t)b[3] << 8 | b[4]);
    if (flags)    *flags    = (uint16_t)((uint16_t)b[6] << 8 | b[7]);

    return ESP_OK;
}