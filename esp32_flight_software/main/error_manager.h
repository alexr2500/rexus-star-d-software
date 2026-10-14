#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/* Bit positions live in protocol.h (ERR_BIT_*) because the Pi sets some
   of them. Descriptions and latching behaviour: main/telemetry_flags.md */
#include "protocol.h"

#define ERR_LATCH_MASK ( (1u << ERR_BIT_PT100)               \
                       | (1u << ERR_BIT_INT_BME)             \
                       | (1u << ERR_BIT_EXT_BME)             \
                       | (1u << ERR_BIT_SLF3S)               \
                       | (1u << ERR_BIT_ABP)                 \
                       | (1u << ERR_BIT_LSM6DSM)             \
                       | (1u << ERR_BIT_HEATER_FAULT)        \
                       | (1u << ERR_BIT_FLUIDIC_FAULT)       \
                       | (1u << ERR_BIT_RXSM_LINK)           \
                       | (1u << ERR_BIT_UNEXPECTED_SIGNAL)   \
                       | (1u << ERR_BIT_MODE_TRANSITION)     \
                       | (1u << ERR_BIT_WATCHDOG_RESET)      \
                       | (1u << ERR_BIT_CSV_WRITE)           \
                       | (1u << ERR_BIT_CONFIG_LOAD) )

/* Bits 21-31 are reserved and transmitted as zero (telemetry_flags.md).
   Nothing may latch there. Caught at compile time, not in flight. */
//0xFFE00000u is bits 21-31 set. An & with the latch mask would output a 1 if any bit >= 21 is set. If so, the static assert fails and the compile fails with message
_Static_assert((ERR_LATCH_MASK & 0xFFE00000u) == 0u,     
               "ERR_LATCH_MASK covers a reserved bit");  //Does not run actual code, is just an assessment that states error during compile if condition is not met

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
                            bool pre_LO);

#endif