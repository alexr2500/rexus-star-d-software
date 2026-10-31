#include "imu_driver.h"
#include "board_pins.h"
#include "driver/spi_master.h"

/* --- LSM6DSM register map --------------------------------------------
 * VERIFY against the LSM6DSM datasheet register table. */
#define LSM_WHO_AM_I      0x0F
#define LSM_WHO_AM_I_VAL  0x6A
#define LSM_CTRL1_XL      0x10
#define LSM_CTRL2_G       0x11
#define LSM_CTRL3_C       0x12
#define LSM_OUTX_L_G      0x22   /* gyro X..Z, 6 bytes, auto-increment  */
#define LSM_OUTX_L_XL     0x28   /* accel X..Z, 6 bytes                 */
#define LSM_READ_FLAG     0x80   /* set bit 7 of the address to read    */

/* CTRL1_XL: ODR_XL[7:4] = 0110 (416 Hz), FS_XL[3:2] = 01 (+/-16 g).
 * Note the unusual encoding - on this part 01 is 16 g, not 4 g. That is
 * the line to check first if every acceleration reads 4x wrong. */
#define LSM_CTRL1_XL_VAL  0x64

/* CTRL2_G: ODR_G[7:4] = 0110 (416 Hz), FS_G[3:2] = 11 (2000 dps). */
#define LSM_CTRL2_G_VAL   0x6C

/* CTRL3_C: BDU (bit 6) + IF_INC (bit 2).
 * BDU matters: without it, the high and low bytes of one axis can come
 * from different samples, which produces occasional wild values that
 * look like vibration. */
#define LSM_CTRL3_C_VAL   0x44

static spi_device_handle_t dev;


static bool lsm_read(uint8_t reg, uint8_t *buf, size_t len)
{
    spi_transaction_t t = {
        .addr      = reg | LSM_READ_FLAG,
        .length    = len * 8,
        .rxlength  = len * 8,
        .rx_buffer = buf,
    };
    return spi_device_polling_transmit(dev, &t) == ESP_OK;
}


bool imu_init(void)
{
    /* TODO: spi_bus_add_device() with PIN_CS_LSM6DSM.
       LSM6DSM is SPI mode 3 (CPOL=1, CPHA=1) - VERIFY.

       Then: read WHO_AM_I and require 0x6A. telemetry_flags.md bit 5
       names a WHO_AM_I mismatch as a fault, and this is the only place
       it can be detected. Then write CTRL3_C, CTRL1_XL, CTRL2_G. */
    return false;
}


bool imu_read(int16_t accel[3], int16_t gyro[3])
{
    uint8_t g[6], a[6];

    if (!lsm_read(LSM_OUTX_L_G, g, sizeof(g))) { return false; }
    if (!lsm_read(LSM_OUTX_L_XL, a, sizeof(a))) { return false; }

    /* Little-endian on the wire, low byte first. */
    for (int i = 0; i < 3; i++) {
        gyro[i]  = (int16_t)(((uint16_t)g[2*i + 1] << 8) | g[2*i]);
        accel[i] = (int16_t)(((uint16_t)a[2*i + 1] << 8) | a[2*i]);
    }
    return true;
}