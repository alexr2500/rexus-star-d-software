#include "mode_manager.h"

static software_mode_t mode;

software_mode_t software_mode_get(void)
{
    return mode;
}