#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "driver/gpio.h"
#include "driver/spi_master.h"

/* Every pin number on the board, in one file.
 *
 * Names match the nets in Main_PCB_STAR_D so a pin can be traced from
 * code to schematic without guessing. Scattering pin numbers through
 * driver files is how a board revision turns into a week of debugging.
 *
 * TODO: every value below is a placeholder. Fill them from the
 * schematic in one sitting, then never touch them again.
 */

/* --- sensor SPI bus (nets ACC_sck / ACC_mosi / ACC_miso) ------------- */
#define SENSOR_SPI_HOST     SPI2_HOST
#define PIN_SPI_SCK         GPIO_NUM_NC   /* TODO ACC_sck   */
#define PIN_SPI_MOSI        GPIO_NUM_NC   /* TODO ACC_mosi  */
#define PIN_SPI_MISO        GPIO_NUM_NC   /* TODO ACC_miso  */

/* --- chip selects ---------------------------------------------------- */
#define PIN_CS_BME_INT      GPIO_NUM_NC   /* TODO CS_BME280      (inside PV) */
#define PIN_CS_BME_EXT      GPIO_NUM_NC   /* TODO CS_BME280_OUT  (outside)   */
#define PIN_CS_MAX31865     GPIO_NUM_NC   /* TODO CS_MAX31865                */
#define PIN_CS_LSM6DSM      GPIO_NUM_NC   /* TODO CS_LSM6DSM                 */
#define PIN_CS_ABP          GPIO_NUM_NC   /* TODO CS_ABP                     */
#define PIN_CS_IOEXP        GPIO_NUM_NC   /* TODO CS_IOEXP - an I/O expander
                                             exists on the board and is not
                                             in the software design at all.
                                             Decide what it is for. */

/* --- I2C (SLF3S flow sensor only) ------------------------------------ */
#define PIN_I2C_SDA         GPIO_NUM_NC   /* TODO I2C_SDA */
#define PIN_I2C_SCL         GPIO_NUM_NC   /* TODO I2C_SCL */

/* --- heaters: three independent channels ----------------------------- */
#define PIN_HEATER1_EN      GPIO_NUM_NC   /* TODO HEATER1_EN */
#define PIN_HEATER2_EN      GPIO_NUM_NC   /* TODO HEATER2_EN */
#define PIN_HEATER3_EN      GPIO_NUM_NC   /* TODO HEATER3_EN */

/* --- stepper --------------------------------------------------------- */
#define PIN_EN_STEPPERS     GPIO_NUM_NC   /* TODO EN_STEPPERS */
#define PIN_DIR_M1          GPIO_NUM_NC   /* TODO DIR_M1      */

/* --- RXSM discretes (owned by rs422_interface, listed here for one
       complete picture of the pinout) --------------------------------- */
#define PIN_RXSM_LO         GPIO_NUM_NC   /* TODO RXSM_LO,   active LOW */
#define PIN_RXSM_SOE        GPIO_NUM_NC   /* TODO RXSM_SOE,  active LOW */
#define PIN_RXSM_SODS       GPIO_NUM_NC   /* TODO RXSM_SODS, active LOW */

/* Open-collector switches that sink current on activation (REXUS ICD
   2.2), so an asserted signal reads LOW. The schematic already has
   R_PULL_LO / R_PULL_SODS; confirm all three have pull-ups and that
   the driver inverts. */

#endif /* BOARD_PINS_H */