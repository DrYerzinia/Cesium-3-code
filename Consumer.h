#ifndef __CONSUMER_H_
#define __CONSUMER_H_

#include <stdint.h>

#include "Producer.h"

typedef enum {

    IDLE,
    BUSY

} Consumer_State;

typedef struct {

    Producer * producer;
    uint16_t packet_list_offset;

} Producer_Consumer_State;

typedef struct {

    Consumer_State state;
    Producer_Consumer_State * producer_list;

} Consumer;

#endif
