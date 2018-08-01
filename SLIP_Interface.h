#ifndef __SLIP_INTERFACE_H_
#define __SLIP_INTERFACE_H_

#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#include "Packet.h"

#include "Producer.h"
#include "Consumer.h"

typedef void (*slip_write)(uint8_t data);

typedef enum {
    START,
    NORMAL,
    ESCAPE,
    END
} SLIP_STATE;

typedef struct {

    SLIP_STATE tx_state;
    SLIP_STATE rx_state;

    slip_write write;

    Packet_Info * tx_pinfo;
    uint8_t * tx_data;
    uint16_t tx_count;

    Consumer * tx;
    Producer * rx;

} SLIP_Interface;

void SLIP_init(SLIP_Interface * slip, slip_write w, Consumer * c, Producer * p);

void SLIP_start(SLIP_Interface * slip, Packet_Info * pinfo, uint8_t *data);

void SLIP_RX_produce_byte(SLIP_Interface * slip, uint8_t byte);
void SLIP_TX_consume_byte(SLIP_Interface * slip);

#endif
