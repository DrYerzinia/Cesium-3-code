#ifndef PACKET_MANAGER_H_
#define PACKET_MANAGER_H_

#include <stdint.h>
#include <string.h>

#include "driverlib.h"

#include "UHF_Radio.h"

#include "Producer.h"
#include "Consumer.h"

#include "SLIP_Interface.h"

extern uint16_t uhf_tx_packet_counter;
extern uint16_t uhf_rx_packet_counter;

extern Consumer UHF_TX;
extern Producer UHF_RX;

void Packet_Manger_init();
void Packet_Manger_process();

void UHF_RX_produce_packet(uint8_t *data, uint16_t len, bool finish);

void UHF_TX_consume_data();

#endif /* PACKET_MANAGER_H_ */
