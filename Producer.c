#include "Producer.h"

void advance_producer(Producer * p){

    uint16_t plen = p->write_location - p->last_pkt_start_location;
    if(plen == 0) return;

    // Set new packet parameters in new packet list
    p->packet_list[p->packet_list_offset].packet_offset = p->last_pkt_start_location;
    p->packet_list[p->packet_list_offset].packet_length = plen;
    p->packet_list[p->packet_list_offset].state = FRESH;

    // Check to see if we have room for another packet before end of buffer and prepare to start next packet
    if(p->buffer_length-p->write_location < MAX_PACKET_SIZE){
        p->write_location = 0;
    }
    p->last_pkt_start_location = p->write_location;

    // Advance the packet list and allow consumers to access last packet
    p->packet_list_offset++;
    if(p->packet_list_offset == p->packet_list_length){ // Wrap the packet list
        p->packet_list_offset = 0;
    }

    // TODO: will be difficult to check overflow.  Will have to advance buffer to find
    // first non consumed packet in list and see its offset to see if we are going to
    // overrun it

}
