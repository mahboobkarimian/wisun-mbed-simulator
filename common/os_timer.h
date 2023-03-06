/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2021, Silicon Labs
 * Main authors:
 *     - Jérôme Pouiller <jerome.pouiller@silabs.com>
 */
#ifndef OS_TIMER_H
#define OS_TIMER_H

#include "nanostack-event-loop/eventOS_callback_timer.h"
#include "slist.h"

struct callback_timer {
    int fd;
    void (*fn)(int, uint16_t);
    struct slist node;
};

int eventOS_callback_timer_expired(int ns_timer_id);
int eventOS_callback_timer_start_sec(int ns_timer_id, int sec);
int eventOS_callback_timer_start_usec(int ns_timer_id, int usec);
int eventOS_callback_timer_expired_usec(int ns_timer_id);
#endif
