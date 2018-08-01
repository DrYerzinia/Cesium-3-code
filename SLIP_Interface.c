#include "SLIP_Interface.h"

#include <stddef.h>

void SLIP_init(SLIP_Interface * slip, slip_write w, Consumer * c, Producer * p){

    slip->tx_state = START;
    slip->rx_state = START;

    slip->write = w;

    slip->tx_pinfo = NULL;
    slip->tx_data = NULL;
    slip->tx_count = 0;

    slip->tx = c;
    slip->rx = p;

}

void SLIP_start(SLIP_Interface * slip, Packet_Info * pinfo, uint8_t *data){

    slip->tx_pinfo = pinfo;
    slip->tx_data = data;

    slip->tx_count = 0;

    slip->tx_state = START;

    SLIP_TX_consume_byte(slip);

}

void SLIP_RX_produce_byte(SLIP_Interface * slip, uint8_t byte){

    switch(slip->rx_state){
        case START:
        case NORMAL:
            switch(byte){
                case SLIP_END:
                    slip->rx_state = NORMAL;
                    advance_producer(slip->rx); // External function
                    break;
                case SLIP_ESC:
                    slip->rx_state = ESCAPE;
                    break;
                default:
                    slip->rx->buffer[slip->rx->write_location++] = byte;
                    break;
            }
            break;
        case ESCAPE:
            switch(byte){
                case SLIP_ESC_END:
                    slip->rx->buffer[slip->rx->write_location++] = SLIP_END;
                    break;
                case SLIP_ESC_ESC:
                    slip->rx->buffer[slip->rx->write_location++] = SLIP_ESC;
                    break;
                default:
                    // TODO ERROR STATE
                    break;
            }
            slip->rx_state = NORMAL;
            break;

    }

}

void SLIP_TX_consume_byte(SLIP_Interface * slip){

    if(slip->tx_pinfo == 0) return;

    if(slip->tx_state == START){
        slip->write(SLIP_END);
        slip->tx_state = NORMAL;
    }

    if(slip->tx_count == slip->tx_pinfo->packet_length){

        if(slip->tx_state == NORMAL){

            slip->write(SLIP_END);
            slip->tx_state = END;
            return;

        } else if(slip->tx_state == END){

            slip->tx_pinfo = 0;
            slip->tx->state = IDLE;
            // TODO mark packet as consumed

        }

        return;

    }

    uint8_t byte = slip->tx_data[slip->tx_pinfo->packet_offset+slip->tx_count];

    switch(slip->tx_state){
        case NORMAL:
            switch(byte){
                case SLIP_END:
                case SLIP_ESC:
                    slip->tx_state = ESCAPE;
                    slip->write(SLIP_ESC);
                    break;
                default:
                    slip->write(byte);
                    slip->tx_count++;
                    break;
            }
            break;
        case ESCAPE:
            switch(byte){
                case SLIP_END:
                    slip->write(SLIP_ESC_END);
                    break;
                case SLIP_ESC:
                    slip->write(SLIP_ESC_ESC);
                    break;
            }
            slip->tx_count++;
            slip->tx_state = NORMAL;
            break;

    }

}

