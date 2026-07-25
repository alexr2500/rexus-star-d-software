#include "rs422_interface.h"
#include <stdio.h>

static bool LO_received = false;        //signals are declared static so that no other parts of the code can affect signal status
static bool SOE_received = false;
static bool SODS_received = false;

void rs422_init(void)
{

}

void rs422_receive(void)
{

}

bool rs422_LO_get(void)
{
    bool LO_event = LO_received;
    LO_received = false;
    return LO_event;
}

bool rs422_SOE_get(void)
{
    return SOE_received;
}

bool rs422_SODS_get(void)
{
    return SODS_received;
}