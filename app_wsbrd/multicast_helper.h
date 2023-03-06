#ifndef MULTICAST_HELPER_H
#define MULTICAST_HELPER_H

#include "mbed-client-libservice/ns_trace.h"
#include "nanostack-event-loop/eventOS_event.h"
#include "nanostack-event-loop/eventOS_scheduler.h"
#include <string.h>

#define multicast_addr_str "ff15::810a:64d1"
extern uint8_t multi_cast_addr[16];

void mul_tasklet(int timer_id, uint16_t slots);
void multicast_start();
#endif