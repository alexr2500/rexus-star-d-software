#include "imu_driver.h"
#include "board_pins.h"
#include "spi_interface.h"
#include "driver/gpio.h"
#include <stddef.h>
#include <string.h>

/* ---- LSM6DSM register map -------------------------------------------- */
#define LSM_REG_INT1_CTRL   0x0D
#define LSM_REG_WHO_AM_I    0x0F
#define LSM_REG_CTRL1_XL    0x10
#define LSM_REG_CTRL2_G     0x11
#define LSM_REG_CTRL3_C     0x12
#define LSM_REG_OUTX_L_G    0x22   /* gyro X..Z then accel X..Z, 12 bytes */
#define LSM_READ_BIT        0x80

#define LSM_WHO_AM_I_VALUE  0x6A

/* CTRL1_XL = 0x64: ODR_XL=0110 (416 Hz), FS_XL=01, BW=00.
   On the LSM6DSM, FS_XL=01 is +-16 g, not +-4 g. The encoding is not in
   increasing order on this part, which is an easy way to end up with a
   silently wrong scale factor. */
#define LSM_CTRL1_XL_VAL    0x64

/* CTRL2_G = 0x6C: ODR_G=0110 (416 Hz), FS_G=11 (+-2000 dps), FS_125=0. */
#define LSM_CTRL2_G_VAL     0x6C

/* CTRL3_C = 0x44: BDU=1 (bit 6), IF_INC=1 (bit 2). */
#define LSM_CTRL3_C_VAL     0x44

/* INT1_CTRL = 0x01: accelerometer data-ready on INT1. The gyro runs at
   the same ODR, so one flag is enough and avoids a double-rate pin. */
#define LSM_INT1_CTRL_VAL   0x01

#define IMU_SPI_CLOCK_HZ    8000000   /* LSM6DSM max is 10 MHz */
#define IMU_SPI_MODE        3         /* CPOL=1, CPHA=1 */

static spi_device_handle_t dev;
static uint8_t whoami;

static esp_err_t reg_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_polling_transmit(dev, &t);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t n)
{
    uint8_t tx[1 + 12] = { (uint8_t)(reg | LSM_READ_BIT) };
    uint8_t rx[1 + 12] = { 0 };
    if (n > 12) return ESP_ERR_INVALID_ARG;

    spi_transaction_t t = {
        .length    = 8u * (n + 1u),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(dev, &t);
    if (err != ESP_OK) return err;

    memcpy(buf, &rx[1], n);     /* byte 0 is clocked out during the address */
    return ESP_OK;
}

esp_err_t imu_driver_init(void)
{
    esp_err_t err = spi_interface_add_device(PIN_CS_LSM6DSM,
                                             IMU_SPI_CLOCK_HZ,
                                             IMU_SPI_MODE, &dev);
    if (err != ESP_OK) return err;

    /* INT1 is an input to us. No pull: the LSM6DSM drives it push-pull. */
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_INT_LSM6DSM),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    err = reg_read(LSM_REG_WHO_AM_I, &whoami, 1);
    if (err != ESP_OK)                  return err;
    if (whoami != LSM_WHO_AM_I_VALUE)   return ESP_ERR_NOT_FOUND;

    /* CTRL3_C first: IF_INC has to be on before any multi-byte read, and
       BDU has to be on before the first sample is latched. */
    err = reg_write(LSM_REG_CTRL3_C,  LSM_CTRL3_C_VAL);
    if (err != ESP_OK) return err;
    err = reg_write(LSM_REG_CTRL1_XL, LSM_CTRL1_XL_VAL);
    if (err != ESP_OK) return err;
    err = reg_write(LSM_REG_CTRL2_G,  LSM_CTRL2_G_VAL);
    if (err != ESP_OK) return err;

    return reg_write(LSM_REG_INT1_CTRL, LSM_INT1_CTRL_VAL);
}

uint8_t imu_driver_whoami(void) { return whoami; }

bool imu_driver_data_ready(void)
{
    return gpio_get_level(PIN_INT_LSM6DSM) != 0;
}

esp_err_t imu_driver_read(imu_sample_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t b[12];
    esp_err_t err = reg_read(LSM_REG_OUTX_L_G, b, sizeof b);
    if (err != ESP_OK) return err;

    /* Little-endian, gyro first, then accelerometer. The cast to int16_t
       happens after the bytes are assembled in an unsigned type: shifting
       a signed value into the sign bit is undefined. */
    out->gx = (int16_t)((uint16_t)b[1]  << 8 | b[0]);
    out->gy = (int16_t)((uint16_t)b[3]  << 8 | b[2]);
    out->gz = (int16_t)((uint16_t)b[5]  << 8 | b[4]);
    out->ax = (int16_t)((uint16_t)b[7]  << 8 | b[6]);
    out->ay = (int16_t)((uint16_t)b[9]  << 8 | b[8]);
    out->az = (int16_t)((uint16_t)b[11] << 8 | b[10]);

    return ESP_OK;
}