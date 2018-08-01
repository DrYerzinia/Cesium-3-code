#ifndef __PACKET_H_
#define __PACKET_H_

#include <stdint.h>

#define MAX_PACKET_SIZE 256

typedef enum {

    FRESH,
    BEING_CONSUMED,
    CONSUMED

} Packet_State;

typedef struct {

    Packet_State state;
    uint16_t packet_offset;
    uint16_t packet_length;

} Packet_Info;

#endif
