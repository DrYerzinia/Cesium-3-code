#include "Packet_Manager.h"

#include "IP_Static.h"

#include "CDH_SLIP.h"
#include "EPS_SLIP.h"

#include "EPS_HnS.h"

#include "util.h"

// Debug

typedef enum {
    DISABLED,
    SELF,
    OTHER
} Loopback_Mode;

Loopback_Mode slip_loopback_mode = DISABLED;
Loopback_Mode radio_loopback_mode = DISABLED;

#define COMMAND_LED_OFF    0x00
#define COMMAND_LED_ON     0x01
#define COMMAND_EPS_BEACON 0x02
#define COMMAND_CDH_ON     0x03
#define COMMAND_ENTER_BSL  0xA3

// Producer indexes for consumers
#define CDH_SLIP_TX_PRODUCER_COUNT 4
#define CDH_SLIP_TX_to_CDH_SLIP_RX 0
#define CDH_SLIP_TX_to_UHF_RX 1

#define Internal_Message_PRODUCER_COUNT 4
#define CDH_SLIP_RX_to_Internal_Message 0
#define EPS_SLIP_RX_to_Internal_Message 1
#define UHF_RX_to_Internal_Message 2

#define UHF_TX_PRODUCER_COUNT 4
#define CDH_SLIP_RX_to_UHF_TX 0
#define Internal_Message_to_UHF_TX 1

#define EPS_TX_PRODUCER_COUNT 1
#define Internal_Message_to_EPS_SLIP_TX 0

Producer CDH_SLIP_RX;
Producer UHF_RX;
Producer SBAND_RX;
Producer Internal_Message_Prod;
Producer EPS_SLIP_RX;

Consumer CDH_SLIP_TX;
Consumer UHF_TX;
Consumer SBAND_TX;
Consumer Internal_Message_Cons;
Consumer EPS_SLIP_TX;

#define CDH_SLIP_RX_BUFFER_SIZE 8192
#define CDH_SLIP_RX_PACKET_LIST_SIZE 32

#pragma PERSISTENT(CDH_SLIP_RX_buffer)

uint8_t CDH_SLIP_RX_buffer[CDH_SLIP_RX_BUFFER_SIZE] = {0};

uint8_t UHF_RX_buffer[512];
uint8_t SBAND_RX_buffer[512];
uint8_t Internal_Message_buffer[256];
uint8_t EPS_SLIP_RX_buffer[256];

Packet_Info CDH_SLIP_RX_packet_list[CDH_SLIP_RX_PACKET_LIST_SIZE];
Packet_Info SBAND_RX_packet_list[5];
Packet_Info UHF_RX_packet_list[5];
Packet_Info Internal_Message_packet_list[2];
Packet_Info EPS_SLIP_RX_list[2];

Producer_Consumer_State CDH_SLIP_TX_producer_list[CDH_SLIP_TX_PRODUCER_COUNT];
Producer_Consumer_State UHF_TX_producer_list[UHF_TX_PRODUCER_COUNT];
Producer_Consumer_State Internal_Message_producer_list[Internal_Message_PRODUCER_COUNT];
Producer_Consumer_State EPS_SLIP_TX_producer_list[EPS_TX_PRODUCER_COUNT];

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

    EPS_SLIP_RX.buffer = EPS_SLIP_RX_buffer;
    EPS_SLIP_RX.buffer_length = 256;
    EPS_SLIP_RX.packet_list = EPS_SLIP_RX_list;
    EPS_SLIP_RX.packet_list_length = 2;
    EPS_SLIP_RX.packet_list_offset = 0;
    EPS_SLIP_RX.write_location = 0;

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
    Internal_Message_producer_list[EPS_SLIP_RX_to_Internal_Message].producer = &EPS_SLIP_RX;
    Internal_Message_producer_list[EPS_SLIP_RX_to_Internal_Message].packet_list_offset = 0;
    Internal_Message_producer_list[UHF_RX_to_Internal_Message].producer = &UHF_RX;
    Internal_Message_producer_list[UHF_RX_to_Internal_Message].packet_list_offset = 0;

    UHF_TX.producer_list = UHF_TX_producer_list;
    UHF_TX.state = IDLE;
    UHF_TX_producer_list[CDH_SLIP_RX_to_UHF_TX].producer = &CDH_SLIP_RX;
    UHF_TX_producer_list[CDH_SLIP_RX_to_UHF_TX].packet_list_offset = 0;
    UHF_TX_producer_list[Internal_Message_to_UHF_TX].producer = &Internal_Message_Prod;
    UHF_TX_producer_list[Internal_Message_to_UHF_TX].packet_list_offset = 0;

    EPS_SLIP_TX.producer_list = EPS_SLIP_TX_producer_list;
    EPS_SLIP_TX.state = IDLE;
    EPS_SLIP_TX_producer_list[Internal_Message_to_EPS_SLIP_TX].producer = &Internal_Message_Prod;
    EPS_SLIP_TX_producer_list[Internal_Message_to_EPS_SLIP_TX].packet_list_offset = 0;

    CDH_SLIP_init(&CDH_SLIP_TX, &CDH_SLIP_RX);
    EPS_SLIP_init(&EPS_SLIP_TX, &EPS_SLIP_RX);

}

void produce_packet(Producer * p, uint8_t *data, uint16_t len, bool finish){

    memcpy(&(p->buffer[p->write_location]), data, len);
    p->write_location += len;
    if(finish){
      advance_producer(p);
    }

}

void Internal_Message_produce_packet(uint8_t *data, uint16_t len, bool finish){

    produce_packet(&Internal_Message_Prod, data, len, finish);

}

void UHF_RX_produce_packet(uint8_t *data, uint16_t len, bool finish){

    produce_packet(&UHF_RX, data, len, finish);

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

void advance_pcs(Producer_Consumer_State * pcs){
    pcs->packet_list_offset++;
    if(pcs->packet_list_offset == pcs->producer->packet_list_length){
        pcs->packet_list_offset = 0;
    }
}

bool consume(Consumer * consumer, int producer_index, Consumer_Data * cd){

    Producer_Consumer_State * pcs = &(consumer->producer_list[producer_index]);
    while(pcs->packet_list_offset != pcs->producer->packet_list_offset){

        Packet_Info * pinfo = &(pcs->producer->packet_list[pcs->packet_list_offset]);

        // If fresh
        if(pinfo->state == FRESH){

            cd->pinfo = pinfo;
            cd->pcs = pcs;
            return true;

        }

        advance_pcs(pcs);

    }

    return false;

}

static inline void CDH_SLIP_TX_consume_CDH_SLIP_RX(){

    // Loopback debug disabled or SLIP TX currently DMAing anything
    if(slip_loopback_mode != SELF || CDH_SLIP_TX.state != IDLE) return;

    Consumer_Data cd = {0};

    if(consume(&CDH_SLIP_TX, CDH_SLIP_TX_to_CDH_SLIP_RX, &cd)){

        // Starts sending packet through UART
        SLIP_start(&CDH_SLIP, cd.pinfo, cd.pcs->producer->buffer);

        cd.pinfo->state = BEING_CONSUMED;
        CDH_SLIP_TX.state = BUSY;

        advance_pcs(cd.pcs);

    }
}

void CDH_SLIP_TX_consume_UHF_RX(){

    // SLIP TX not currently DMAing anything
    if(CDH_SLIP_TX.state != IDLE) return;

    Consumer_Data cd = {0};

    while(consume(&CDH_SLIP_TX, CDH_SLIP_TX_to_UHF_RX, &cd)){

        uint16_t offset = cd.pinfo->packet_offset;
        uint8_t * data = cd.pcs->producer->buffer;

        if(memcmp(data+offset+12, internal_ip, 4) != 0){ // Check that IP address not meant for internal

            // Starts sending packet through UART
            SLIP_start(&CDH_SLIP, cd.pinfo, cd.pcs->producer->buffer);

            cd.pinfo->state = BEING_CONSUMED;
            CDH_SLIP_TX.state = BUSY;

        }

        advance_pcs(cd.pcs);

    }
}

void UHF_TX_consume_CDH_SLIP_RX(){

    // Conditions for execution
    // 1. Not currently RXing packet, checked by RADIO state not RX_PARITAL, RX_COMPLETE
    //      TODO this breaks beginings of packets, consider changing state on Preamble/Sync detect
    // 2. Not currently TXing packet (solved by BUSY state), ends when last data written to buffer
    if(uhf_radio_state == RX_PARTIAL || uhf_radio_state == RX_DONE) return;

    if(UHF_TX.state == BUSY) return;

    Consumer_Data cd = {0};

    while(consume(&UHF_TX, CDH_SLIP_RX_to_UHF_TX, &cd)){

        uint16_t offset = cd.pinfo->packet_offset;
        uint8_t * data = cd.pcs->producer->buffer;

        if(memcmp(data+offset+12, internal_ip, 4) != 0){ // Check that IP address not meant for internal

            UHF_TX.state = BUSY;
            cd.pinfo->state = BEING_CONSUMED;

            UHF_TX_pinfo = cd.pinfo;
            UHF_TX_data = data+offset;
            UHF_TX_count = 0;
            if(uhf_radio_state != TX){
                UHF_TX_consume_data();
            }

            break;

        }

        advance_pcs(cd.pcs);

    }

}

UHF_TX_consume_Internal_Message(){

    if(uhf_radio_state == RX_PARTIAL || uhf_radio_state == RX_DONE) return;

    if(UHF_TX.state == BUSY) return;

    Consumer_Data cd = {0};

    while(consume(&UHF_TX, Internal_Message_to_UHF_TX, &cd)){

        uint16_t offset = cd.pinfo->packet_offset;
        uint8_t * data = cd.pcs->producer->buffer;

        if(*(data+offset) == 2){ // Opcode for Internal to UHF, using as TNC port number where EPS in port 2

             UHF_TX.state = BUSY;
             cd.pinfo->state = BEING_CONSUMED;

             UHF_TX_pinfo = cd.pinfo;
             UHF_TX_data = data+offset;
             UHF_TX_count = 0;
             if(uhf_radio_state != TX){
                 UHF_TX_consume_data();
             }

             break;

        }

        advance_pcs(cd.pcs);

    }

}

void Internal_Message_consume_CDH_SLIP_RX(){

    if(slip_loopback_mode != DISABLED) return;

    Consumer_Data cd = {0};

    while(consume(&Internal_Message_Cons, CDH_SLIP_RX_to_Internal_Message, &cd)){

        uint16_t offset = cd.pinfo->packet_offset;
        uint8_t * data = cd.pcs->producer->buffer;

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

            cd.pinfo->state = CONSUMED;

        }

        advance_pcs(cd.pcs);

    }
}

void Internal_Message_consume_UHF_RX(){

    // TODO UHF loopback check
    //if(slip_loopback_mode != DISABLED) return;

    Consumer_Data cd = {0};

    while(consume(&Internal_Message_Cons, UHF_RX_to_Internal_Message, &cd)){

        uint16_t offset = cd.pinfo->packet_offset;
        uint8_t * data = cd.pcs->producer->buffer;

        if(memcmp(data+offset+12, internal_ip, 4) == 0){ // Check that IP address matches target

            // Get command byte
            uint8_t cmd = data[offset+28];
            switch(cmd){
                case COMMAND_EPS_BEACON:
                    {
                        // Read beacon count value
                        memcpy(&EPS_HnS_beacon_count, offset+28+1, sizeof(EPS_HnS_beacon_count));
                    }
                    break;
                case COMMAND_CDH_ON:
                    {
                        // TODO in multiple interrupts, need to make sure non-reenerable
                        // Sends a message to EPS
                        uint8_t com_on_command[1] = {0x01};
                        Internal_Message_produce_packet(com_on_command, 1, true);
                    }
                    break;

            }

            cd.pinfo->state = CONSUMED;

        }

        advance_pcs(cd.pcs);

    }
}

EPS_SLIP_TX_consume_Internal_Message(){

    // SLIP TX not currently DMAing anything
    if(EPS_SLIP_TX.state != IDLE) return;

    Consumer_Data cd = {0};

    while(consume(&EPS_SLIP_TX, Internal_Message_to_EPS_SLIP_TX, &cd)){

        uint16_t offset = cd.pinfo->packet_offset;
        uint8_t * data = cd.pcs->producer->buffer;

        if(*(data+offset) == 1){ // Opcode for COM to EPS, using as TNC port number where EPS in port 1

            // Starts sending packet through UART
            SLIP_start(&EPS_SLIP, cd.pinfo, cd.pcs->producer->buffer);

            cd.pinfo->state = BEING_CONSUMED;
            EPS_SLIP_TX.state = BUSY;

        }

        advance_pcs(cd.pcs);

    }

}

Internal_Message_consume_EPS_SLIP_RX(){

     Consumer_Data cd = {0};

     while(consume(&Internal_Message_Cons, EPS_SLIP_RX_to_Internal_Message, &cd)){

         uint16_t offset = cd.pinfo->packet_offset;
         uint8_t * data = cd.pcs->producer->buffer;

         if(*(data+offset) == 0){ // Opcode for H&S from EPS to COM

             // TODO load H&S buffer
             memcpy(EPS_HnS, data+offset+1, cd.pinfo->packet_length-1);
             EPS_HnS_len = cd.pinfo->packet_length-1;

             // TODO write task to beacon EPS periodically if its turned on
             // TODO write code to consume Internal Message from UHF RX and add EPS H&S enabling code
             //  Probably use #beacons timeout where -1 is never ending

             cd.pinfo->state = CONSUMED;

         }

         advance_pcs(cd.pcs);

     }

}

void Packet_Manger_process(){

    // Our messages to the ground are very important and should happen seldomly

    // 1 ) UHF TX             from   Internal Message
    // 2 ) S-Band TX          from   Internal Message
    UHF_TX_consume_Internal_Message();

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
    Internal_Message_consume_UHF_RX();

    // Loopback test modes lowest priority

    // 11) SLIP TX            from   SLIP RX            (If SLIP Loopback Mode ALL)
    CDH_SLIP_TX_consume_CDH_SLIP_RX();

    // 12) UHF TX             from   UHF RX             (If RADIO Loopback Mode SELF)
    // 13) UHF TX             from   S-Band RX          (If RADIO Loopback Mode OTHER)

    // 14) S-Band TX          from   S-Band RX          (If RADIO Loopback Mode SELF)
    // 15) S-Band TX          from   UHF RX             (If RADIO Loopback Mode OTHER)

    // 17) EPS SLIP TX        from   Internal Message
    // 18) Internal Message   from   EPS SLIP RX

    EPS_SLIP_TX_consume_Internal_Message();
    Internal_Message_consume_EPS_SLIP_RX();

}
