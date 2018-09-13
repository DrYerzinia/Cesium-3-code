#ifndef __PRODUCER_H_
#define __PRODUCER_H_

#include <stdint.h>

#include "Packet.h"

typedef struct {

    uint8_t * buffer;
    uint16_t buffer_length;
    uint16_t write_location;
    uint16_t last_pkt_start_location;

    Packet_Info * packet_list;
    uint16_t packet_list_offset;
    uint16_t packet_list_length;

} Producer;

void advance_producer(Producer * p);
void reset_producer(Producer * p);

#endif
