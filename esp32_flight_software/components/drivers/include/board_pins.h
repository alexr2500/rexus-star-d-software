/* board_pins.h — STAR-D main PCB, REXUS 38
 *
 * Source: VFINAL_PCB_Rexus/Main_PCB_STAR_D.kicad_sch + 5 sub-sheets,
 * traced 2026-11-03. Net names are the schematic's own, verbatim.
 * Cross-checked against SED v3-1 tables 4.5 and 4.13.
 *
 * Items marked REVIEW are open against hardware. They are correct as
 * drawn; they may not be correct as intended.
 */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "driver/gpio.h"

/* ===== sensor SPI bus — ESP32 is MASTER =============================== */
#define PIN_SPI_SCK            GPIO_NUM_13   /* SPI_SCK  */
#define PIN_SPI_MOSI           GPIO_NUM_17   /* SPI_MOSI */
#define PIN_SPI_MISO           GPIO_NUM_45   /* SPI_MISO — REVIEW: GPIO45 is the
                                              * VDD_SPI strap and floats at boot
                                              * with every CS idle. Wants an
                                              * external pull-down. */

#define PIN_CS_BME280_IN       GPIO_NUM_10   /* CS_BME280          -> U2, on-board */
#define PIN_CS_BME280_OUT      GPIO_NUM_11   /* CS_EXTERNAL_BME280 -> J7.6         */
#define PIN_CS_LSM6DSM         GPIO_NUM_12   /* CS_LSM6DSM         -> U4           */
#define PIN_CS_MAX31865        GPIO_NUM_21   /* CS_MAX31865        -> IC1          */
#define PIN_CS_ABP             GPIO_NUM_9    /* CS_ABP             -> J1.5         */

#define PIN_INT_LSM6DSM        GPIO_NUM_18   /* INT_LSM6DSM -> U4.4 INT1 */

/* ===== Raspberry Pi SPI — ESP32 is SLAVE (SED table 4.13) ============= */
#define PIN_RASP_CS            GPIO_NUM_34
#define PIN_RASP_CLK           GPIO_NUM_35
#define PIN_RASP_MOSI          GPIO_NUM_36
#define PIN_RASP_MISO          GPIO_NUM_46   /* ROM-message strap; must not be
                                              * held high at reset by the Pi. */
#define PIN_RASP_HANDSHAKE     (-1)          /* REVIEW: J3 has only 4 pins. No wire
                                              * exists for the handshake. */

/* ===== I2C — SLF3S flow sensor only, off-board on J4 ================== */
#define PIN_I2C_SCL            GPIO_NUM_3    /* J4.5 (JTAG-source strap) */
#define PIN_I2C_SDA            GPIO_NUM_7    /* J4.2                     */

/* ===== RS-422 to the RXSM, via ISL83491 (IC3) ========================= *
 * REVIEW: the schematic wires IC3's RO (an output) to U0TXD (an output)
 * and DI (an input) to U0RXD. That looks swapped. The values below follow
 * the SCHEMATIC, so uart_tcu.c must assign them explicitly through the
 * GPIO matrix and the IDF console must move to USB Serial/JTAG (J10).
 * If hardware swaps the nets, use the default UART0 mapping instead.
 */
#define PIN_RS422_RX           GPIO_NUM_43   /* ESP_32_RO -> IC3.2 RO (as drawn) */
#define PIN_RS422_TX           GPIO_NUM_44   /* ESP_32_DI -> IC3.5 DI (as drawn) */
/* IC3 ~RE tied GND, DE tied 3V3: always receiving, always driving.
 * There is no transmit-enable for software to control. */

/* ===== REXUS service flags ============================================ *
 * REVIEW: requirement D.35 says these shall be optically isolated. This
 * revision has no optocoupler and no pull-up. Polarity below assumes the
 * ICD low-side switch (sinks on activation => active LOW) with the ESP's
 * internal pull-up. Re-adding isolation may invert the sense.
 */
#define PIN_LO_IN              GPIO_NUM_1    /* LO_IN   <- J2.5 */
#define PIN_SOE_IN             GPIO_NUM_2    /* SOE_IN  <- J2.6 */
#define PIN_SODS_IN            GPIO_NUM_14   /* SODS_IN <- J2.7 */
#define REXUS_FLAG_ACTIVE_LEVEL   0

/* ===== heaters ======================================================== *
 * 100R series resistor and a pull-down on every gate, so the heaters are
 * OFF while the pins are high-impedance at reset.
 * NOTE: HEATER3_EN drives TWO MOSFETs (Q9 via R_G3, Q10 via R_G4), so
 * three control lines switch four physical channels.
 */
#define PIN_HEATER1_EN         GPIO_NUM_33   /* -> Q7       */
#define PIN_HEATER2_EN         GPIO_NUM_37   /* -> Q8       */
#define PIN_HEATER3_EN         GPIO_NUM_38   /* -> Q9 + Q10 */
#define HEATER_ACTIVE_LEVEL    1             /* N-MOSFET: high = conducting */

/* ===== steppers ======================================================= */

/* Syringe pump — TMC2209 (IC6). The driver does the commutation. */
#define PIN_PUMP_STEP          GPIO_NUM_5    /* STEP_M1     -> IC6.16 */
#define PIN_PUMP_DIR           GPIO_NUM_6    /* DIR_M1      -> IC6.19 */
#define PIN_PUMP_ENN           GPIO_NUM_8    /* EN_STEPPERS -> IC6.2  */
#define PUMP_ENABLE_LEVEL      0             /* ENN is active LOW */

/* Microscope stage — ULN2003 (IC5). Software generates the sequence.
 * No enable pin: a coil stays energised while its input is high. */
#define PIN_STAGE_IN1          GPIO_NUM_42   /* IC5.1 (1B) -> COIL_A */
#define PIN_STAGE_IN2          GPIO_NUM_41   /* IC5.2 (2B) -> COIL_B */
#define PIN_STAGE_IN3          GPIO_NUM_40   /* IC5.3 (3B) -> COIL_C */
#define PIN_STAGE_IN4          GPIO_NUM_39   /* IC5.4 (4B) -> COIL_D */
/* GPIO39-42 are the JTAG pins: hardware JTAG is unavailable.
 * Debug over USB Serial/JTAG on J10. */

/* ===== analogue ======================================================= */
#define PIN_ADC_POT            GPIO_NUM_4    /* GPIO_ADC -> R_ADC_POT1 + C17.
                                              * Potentiometer (SED fig. 38), NOT a
                                              * bus-voltage monitor. sensors.c's
                                              * voltage_mV has no source here. */

/* No status LEDs on this revision. */

#endif /* BOARD_PINS_H */