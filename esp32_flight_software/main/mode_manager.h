#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include "protocol.h"   /* software_mode_t lives here; the Pi decodes the
                           same values, so this file must not redefine it */

software_mode_t software_mode_get(void);

#endif