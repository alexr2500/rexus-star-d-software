#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

/* Bit positions live in protocol.h (ERR_BIT_*) because the Pi sets some
   of them. Descriptions and latching behaviour: main/telemetry_flags.md */
#include "protocol.h"

#define ERR_LATCH_MASK ( (1u << PT100_ERROR)                \
                       | (1u << INT_BME_ERROR)              \
                       | (1u << EXT_BME_ERROR)              \
                       | (1u << SLF3S_ERROR)                \
                       | (1u << ABP_ERROR)                  \
                       | (1u << LSM6DSM_ERROR)              \
                       | (1u << HEATER_FAULT)               \
                       | (1u << FLUIDIC_FAULT)              \
                       | (1u << RXSM_LINK_FAULT)            \
                       | (1u << UNEXPECTED_SIGNAL_ORDER)    \
                       | (1u << MODE_TRANSITION_FAULT)      \
                       | (1u << WATCHDOG_RESET)             \
                       | (1u << CSV_WRITE_FAULT)            \
                       | (1u << CONFIG_LOAD_FAULT)          \)

void     error_manager_init(void);
void     error_raise(uint8_t bit);
void     error_clear(uint8_t bit);
bool     error_is_set(uint8_t bit);
uint32_t error_flags_get(void);

/* Called on every successful STATUS reply. Derives bits 7, 8 and 9 from
   what the Pi actually sends. Not called when a poll goes unanswered:
   the bits then hold their last value and PI_DEGRADED marks them stale. */
void error_update_pi_status(camera_status_t cam,
                            uint8_t ssd_free_gb,
                            bool pre_lo);

#endif