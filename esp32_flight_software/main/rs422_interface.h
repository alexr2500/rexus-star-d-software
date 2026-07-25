#ifndef RS422_INTERFACE_H
#define RS422_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

void rs422_init();
void rs422_receive();

bool rs422_LO_get();
bool rs422_SOE_get();
bool rs422_SODS_get();

#endif