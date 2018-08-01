#include "Packet_Manager.h"

#include "util.h"

// Debug

typedef enum {
    DISABLED,
    SELF,
    OTHER
} Loopback_Mode;

Loopback_Mode slip_loopback_mode = DISABLED;
Loopback_Mode radio_loopback_mode = DISABLED;

#define COMMAND_LED_OFF   0x00
#define COMMAND_LED_ON    0x01
#define COMMAND_ENTER_BSL 0xA3

// Producer indexes for consumers
#define CDH_SLIP_TX_PRODUCER_COUNT 4
#define CDH_SLIP_TX_to_CDH_SLIP_RX 0
#define CDH_SLIP_TX_to_UHF_RX 1

#define Internal_Message_PRODUCER_COUNT 4
#define CDH_SLIP_RX_to_Internal_Message 0

#define UHF_TX_PRODUCER_COUNT 4
#define CDH_SLIP_RX_to_UHF_TX 0

Producer CDH_SLIP_RX;
Producer UHF_RX;
Producer SBAND_RX;
Producer Internal_Message_Prod;

Consumer CDH_SLIP_TX;
Consumer UHF_TX;
Consumer SBAND_TX;
Consumer Internal_Message_Cons;

#define CDH_SLIP_RX_BUFFER_SIZE 8192
#define CDH_SLIP_RX_PACKET_LIST_SIZE 32

#pragma PERSISTENT(CDH_SLIP_RX_buffer)

uint8_t CDH_SLIP_RX_buffer[CDH_SLIP_RX_BUFFER_SIZE] = {0};

uint8_t UHF_RX_buffer[512];
uint8_t SBAND_RX_buffer[512];
uint8_t Internal_Message_buffer[256];

Packet_Info CDH_SLIP_RX_packet_list[CDH_SLIP_RX_PACKET_LIST_SIZE];
Packet_Info SBAND_RX_packet_list[5];
Packet_Info UHF_RX_packet_list[5];
Packet_Info Internal_Message_packet_list[2];

Producer_Consumer_State CDH_SLIP_TX_producer_list[CDH_SLIP_TX_PRODUCER_COUNT];
Producer_Consumer_State UHF_TX_producer_list[UHF_TX_PRODUCER_COUNT];
Producer_Consumer_State Internal_Message_producer_list[Internal_Message_PRODUCER_COUNT];

SLIP_Interface CDH_SLIP;
SLIP_Interface EPS_SLIP;

void CDH_slip_write(uint8_t data){

        HWREG16(EUSCI_A0_BASE + OFS_UCAxTXBUF) = data;
}

void Packet_Manger_init(){

    // Initalize Producers
    CDH_SLIP_RX.buffer = CDH_SLIP_RX_buffer;
    CDH_SLIP_RX.buffer_length = CDH_SLIP_RX_BUFFER_SIZE;
    CDH_SLIP_RX.last_pkt_start_location = 0;
    CDH_SLIP_RX.packet_list = CDH_SLIP_RX_packet_list;
    CDH_SLIP_RX.packet_list_length = CDH_SLIP_RX_PACKET_LIST_SIZE;
    CDH_SLIP_RX.packet_list_offset = 0;
    CDH_SLIP_RX.write_location = 0;

    UHF_RX.buffer = UHF_RX_buffer;
    UHF_RX.buffer_length = 512;
    UHF_RX.packet_list = UHF_RX_packet_list;
    UHF_RX.packet_list_length = 5;
    UHF_RX.packet_list_offset = 0;
    UHF_RX.write_location = 0;

    SBAND_RX.buffer = SBAND_RX_buffer;
    SBAND_RX.buffer_length = 512;
    SBAND_RX.packet_list = SBAND_RX_packet_list;
    SBAND_RX.packet_list_length = 5;
    SBAND_RX.packet_list_offset = 0;
    SBAND_RX.write_location = 0;

    Internal_Message_Prod.buffer = Internal_Message_buffer;
    Internal_Message_Prod.buffer_length = 256;
    Internal_Message_Prod.packet_list = Internal_Message_packet_list;
    Internal_Message_Prod.packet_list_length = 2;
    Internal_Message_Prod.packet_list_offset = 0;
    Internal_Message_Prod.write_location = 0;

    // Initialize Consumers
    CDH_SLIP_TX.producer_list = CDH_SLIP_TX_producer_list;
    CDH_SLIP_TX.state = IDLE;
    CDH_SLIP_TX_producer_list[CDH_SLIP_TX_to_CDH_SLIP_RX].producer = &CDH_SLIP_RX;
    CDH_SLIP_TX_producer_list[CDH_SLIP_TX_to_CDH_SLIP_RX].packet_list_offset = 0;
    CDH_SLIP_TX_producer_list[CDH_SLIP_TX_to_UHF_RX].producer = &UHF_RX;
    CDH_SLIP_TX_producer_list[CDH_SLIP_TX_to_UHF_RX].packet_list_offset = 0;

    Internal_Message_Cons.producer_list = Internal_Message_producer_list;
    Internal_Message_Cons.state = IDLE;
    Internal_Message_producer_list[CDH_SLIP_RX_to_Internal_Message].producer = &CDH_SLIP_RX;
    Internal_Message_producer_list[CDH_SLIP_RX_to_Internal_Message].packet_list_offset = 0;

    UHF_TX.producer_list = UHF_TX_producer_list;
    UHF_TX.state = IDLE;
    UHF_TX_producer_list[CDH_SLIP_RX_to_UHF_TX].producer = &CDH_SLIP_RX;
    UHF_TX_producer_list[CDH_SLIP_RX_to_UHF_TX].packet_list_offset = 0;

    SLIP_init(&CDH_SLIP, CDH_slip_write, &CDH_SLIP_TX, &CDH_SLIP_RX);


}

void UHF_RX_produce_packet(uint8_t *data, uint16_t len, bool finish){

    memcpy(&(UHF_RX.buffer[UHF_RX.write_location]), data, len);
    UHF_RX.write_location += len;
    if(finish){
      advance_producer(&UHF_RX);
    }

}

Packet_Info * UHF_TX_pinfo = 0;
uint8_t * UHF_TX_data;
uint16_t UHF_TX_count;

#define FIFO_FILL 40

uint16_t uhf_tx_packet_counter = 0;
uint16_t uhf_rx_packet_counter = 0;

void write_fifo(uint16_t len){

    uint16_t to_send = UTIL_MIN(len-UHF_TX_count, FIFO_FILL);
    write_block(&sconf, SPIRIT1_FIFO, UHF_TX_data+UHF_TX_count, to_send);
    UHF_TX_count += to_send;

    // If we reached the end of the packet
    if(UHF_TX_count == len){

        uhf_tx_packet_counter++;

        UHF_TX_pinfo->state = CONSUMED;
        UHF_TX.state = IDLE;

    }

}

void UHF_TX_consume_data(){

    if(UHF_TX_pinfo == 0){
        return; // Check to make sure there is work to be done
    }

    uint16_t len = UHF_TX_pinfo->packet_length;

    // If we are starting the next packet
    if(UHF_TX_count == 0){

        // Set the size
        writereg8(&sconf, SPIRIT1_PCKTLEN1, ((len & 0xFF00) >> 8));
        writereg8(&sconf, SPIRIT1_PCKTLEN0, (len & 0x00FF));

        if(uhf_radio_state != TX){
            //SPIRIT1_flush_tx(&sconf);
        }

        // Write the first data
        write_fifo(len);

        // If we are not in TX, set it up and start
        if(uhf_radio_state != TX){
            UHF_init_TX();
        } else {
            SPIRIT1_enable_tx(&sconf);
        }

    } else {

        // If we are continuing the packet, just write the next bytes
        write_fifo(len);

    }

}

static inline void CDH_SLIP_TX_consume_CDH_SLIP_RX(){

    // Loopback debug enabled
    if(slip_loopback_mode == SELF){

        // SLIP TX not currently DMAing anything
        if(CDH_SLIP_TX.state == IDLE){

            // If we are behind the producer in packets
            Producer_Consumer_State * pcs = &(CDH_SLIP_TX.producer_list[CDH_SLIP_TX_to_CDH_SLIP_RX]);
            while(pcs->packet_list_offset != pcs->producer->packet_list_offset){

                Packet_Info * pinfo = &(pcs->producer->packet_list[pcs->packet_list_offset]);

                // If fresh
                if(pinfo->state == FRESH){

                    // Starts sending packet through UART
                    SLIP_start(&CDH_SLIP, pinfo, pcs->producer->buffer);

                    pinfo->state = BEING_CONSUMED;
                    CDH_SLIP_TX.state = BUSY;

                    break;

                }

                pcs->packet_list_offset++;
                if(pcs->packet_list_offset == pcs->producer->packet_list_length){
                    pcs->packet_list_offset = 0;
                }

            }
        }
    }
}

void CDH_SLIP_TX_consume_UHF_RX(){

    // SLIP TX not currently DMAing anything
    if(CDH_SLIP_TX.state == IDLE){

        // If we are behind the producer in packets
        Producer_Consumer_State * pcs = &(CDH_SLIP_TX.producer_list[CDH_SLIP_TX_to_UHF_RX]);
        while(pcs->packet_list_offset != pcs->producer->packet_list_offset){

            Packet_Info * pinfo = &(pcs->producer->packet_list[pcs->packet_list_offset]);

            // If fresh
            if(pinfo->state == FRESH){

                // Starts sending packet through UART
                SLIP_start(&CDH_SLIP, pinfo, pcs->producer->buffer);

                pinfo->state = BEING_CONSUMED;
                CDH_SLIP_TX.state = BUSY;

                break;

            }

            pcs->packet_list_offset++;
            if(pcs->packet_list_offset == pcs->producer->packet_list_length){
                pcs->packet_list_offset = 0;
            }

        }
    }
}


uint8_t internal_ip[] = {0x01, 0x01, 0x01, 0x03}; // 1.1.1.3
uint8_t ground_ip[]   = {0x01, 0x01, 0x01, 0x02}; // 1.1.1.2

void UHF_TX_consume_CDH_SLIP_RX(){

    // Conditions for execution
    // 1. Not currently RXing packet, checked by RADIO state not RX_PARITAL, RX_COMPLETE
    //      TODO this breaks beginings of packets, consider changing state on Preamble/Sync detect
    // 2. Not currently TXing packet (solved by BUSY state), ends when last data written to buffer
    if(uhf_radio_state == RX_PARTIAL || uhf_radio_state == RX_DONE) return;

    if(UHF_TX.state == BUSY) return;

    Producer_Consumer_State * pcs = &(UHF_TX.producer_list[CDH_SLIP_RX_to_UHF_TX]);
    while(pcs->packet_list_offset != pcs->producer->packet_list_offset){

        Packet_Info * pinfo = &(pcs->producer->packet_list[pcs->packet_list_offset]);

        // If fresh
        if(pinfo->state == FRESH){

            uint16_t offset = pinfo->packet_offset;
            uint8_t * data = pcs->producer->buffer;

            // TODO figure out what actual filtering to use here
            if(memcmp(data+offset+12, internal_ip, 4) != 0){ // Check that IP address not meant for internal
            //if(memcmp(data+offset+12, ground_ip, 4) != 0){ // Check that IP address matches target

                UHF_TX.state = BUSY;
                pinfo->state = BEING_CONSUMED;

                UHF_TX_pinfo = pinfo;
                UHF_TX_data = data+offset;
                UHF_TX_count = 0;
                if(uhf_radio_state != TX){
                    UHF_TX_consume_data();
                }

                break;

            }

        }

        pcs->packet_list_offset++;
        if(pcs->packet_list_offset == pcs->producer->packet_list_length){
            pcs->packet_list_offset = 0;
        }
    }

}


static inline void Internal_Message_consume_CDH_SLIP_RX(){

    if(slip_loopback_mode == DISABLED){

        Producer_Consumer_State * pcs = &(Internal_Message_Cons.producer_list[CDH_SLIP_RX_to_Internal_Message]);
        while(pcs->packet_list_offset != pcs->producer->packet_list_offset){

            Packet_Info * pinfo = &(pcs->producer->packet_list[pcs->packet_list_offset]);

            // If fresh
            if(pinfo->state == FRESH){

                uint16_t offset = pinfo->packet_offset;
                uint8_t * data = pcs->producer->buffer;

                if(memcmp(data+offset+12, internal_ip, 4) == 0){ // Check that IP address matches target

                    // Get command byte
                    uint8_t cmd = data[offset+28];
                    switch(cmd){
                        case COMMAND_LED_ON:
                            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                            break;
                        case COMMAND_LED_OFF:
                            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
                            break;
                        case COMMAND_ENTER_BSL:
                            {
                                __disable_interrupt(); // Ensure no application interrupts fire during BSL
                                // This sends execution to the BSL. When execution
                                // returns to the user app, it will be via the reset
                                // vector, meaning execution will re-start.
                                ((void (*)())0x1000)(); // BSL0
                            }
                            break;
                        default:
                            break;
                    }

                    pinfo->state = CONSUMED;

                }

            }

            pcs->packet_list_offset++;
            if(pcs->packet_list_offset == pcs->producer->packet_list_length){
                pcs->packet_list_offset = 0;
            }

        }

    }
}

void Packet_Manger_process(){

    // Our messages to the ground are very important and should happen seldomly

    // 1 ) UHF TX             from   Internal Message
    // 2 ) S-Band TX          from   Internal Message

    // Clearing TX buffer from CDH is next highest priority

    // 3 ) UHF TX             from   SLIP RX            (Skip if SLIP Loopback Mode ALL)
    // 4 ) S-Band TX          from   SLIP RX            (Skip if SLIP Loopback Mode ALL)
    UHF_TX_consume_CDH_SLIP_RX();

    // We want our messages to CDH to be before radio traffic and they should happen seldomly (Buffer Full Warning)

    // 5 ) SLIP TX            from   Internal Message

    // Clearing out radio RX buffers is important

    // 6 ) SLIP TX            from   S-Band RX          (Skip if RADIO Loopback Mode not NONE)
    // 7 ) SLIP TX            from   UHF RX             (Skip if RADIO Loopback Mode not NONE)
    CDH_SLIP_TX_consume_UHF_RX();

    // Worry about reading in internal messages after things are on there way to radios
    // 8 ) Internal Message   from   SLIP RX            (Skip if SLIP  Loopback Mode ALL)
    // 9 ) Internal Message   from   UHF RX             (Skip if RADIO Loopback Mode not NONE)
    // 10) Internal Message   from   S-Band RX          (Skip if RADIO Loopback Mode not NONE)
    Internal_Message_consume_CDH_SLIP_RX();

    // Loopback test modes lowest priority

    // 11) SLIP TX            from   SLIP RX            (If SLIP Loopback Mode ALL)
    CDH_SLIP_TX_consume_CDH_SLIP_RX();

    // 12) UHF TX             from   UHF RX             (If RADIO Loopback Mode SELF)
    // 13) UHF TX             from   S-Band RX          (If RADIO Loopback Mode OTHER)

    // 14) S-Band TX          from   S-Band RX          (If RADIO Loopback Mode SELF)
    // 15) S-Band TX          from   UHF RX             (If RADIO Loopback Mode OTHER)

}

// CDH SLIP Interface Interrupt
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void){

    uint8_t rx_data;
    switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:

            rx_data = (HWREG16(EUSCI_A0_BASE + OFS_UCAxRXBUF)); // Read UART data fast
            SLIP_RX_produce_byte(&CDH_SLIP, rx_data);
            break;

        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG:
            SLIP_TX_consume_byte(&CDH_SLIP);
            break;
    }
}
