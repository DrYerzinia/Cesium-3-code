#include "Packet_Manager.h"

#include "IP_Static.h"

#include "CDH_SLIP.h"
#include "EPS_SLIP.h"

#include "EPS_HnS.h"

#include "util.h"

///////////// Some definitions for a real IP/UDP stack /////////////////////
typedef struct _IPheader {
    uint8_t ip_hl_v;   /**< Header length and version */
    uint8_t ip_tos;    /**< Type of service */
    uint16_t ip_len;   /**< Total length */
    uint16_t ip_id;    /**< Identification */
    uint16_t ip_off;   /**< Fragment offset field */
    uint8_t ip_ttl;    /**< Time to live */
    uint8_t ip_p;      /**< Protocol */
    uint16_t ip_sum;   /**< Checksum */
    uint8_t ip_src[4]; /**< Source IP address */
    uint8_t ip_dst[4]; /**< Destination IP address */
} ip_header_t, *p_ip_header_t;

typedef struct _UDPheader {
    uint16_t srcport;
    uint16_t destport;
    uint16_t len;
    uint16_t chksum;
} udp_header_t, *p_udp_header_t;

/** Ethernet IP header size */
#define ETH_IP_HEADER_SIZE   (sizeof(ip_header_t))

/** Ethernet UDP header size */
#define ETH_UDP_HEADER_SIZE (sizeof(udp_header_t))

/** Swap 2 bytes of a word */
#define SWAP16(x)   (((x & 0xff) << 8) | ((x & 0xFF00) >> 8))
// Swap 4 bytes -> htonl
#define SWAP32(x) ( ((x & 0xFF000000) >> 24) | ((x & 0x00FF0000) >> 8) | ((x & 0x0000FF00) << 8) | ((x & 0x000000FF) << 24) )

#define IP_PROT_UDP             17

// Debug

typedef enum {
    DISABLED,
    SELF,
    OTHER
} Loopback_Mode;

Loopback_Mode slip_loopback_mode = DISABLED;
Loopback_Mode radio_loopback_mode = DISABLED;

#define COMMAND_LED_OFF       0x00
#define COMMAND_LED_ON        0x01
#define COMMAND_EPS_BEACON    0x02
#define COMMAND_CDH_ON        0x03
#define COMMAND_BSL_PASS      0x04
#define COMMAND_MUTE_UHF_TX   0x05
#define COMMAND_UNMUTE_UHF_TX 0x06
#define COMMAND_CHANGE_BAUD   0x07
#define COMMAND_ENTER_BSL     0xA3

static const uint8_t mute_passwd[32] =
    {
        0x42, 0x76, 0x03, 0x2d, 0x1d, 0x4b, 0x0b, 0xac,
        0xaf, 0x3a, 0xcc, 0x51, 0xaf, 0x7a, 0x1a, 0xe3,
        0xfb, 0x72, 0xd5, 0xfe, 0x74, 0xd0, 0x0c, 0xac,
        0x49, 0x5a, 0xa5, 0x98, 0x90, 0xc7, 0x7f, 0xe0
    };

// Producer indexes for consumers
#define CDH_SLIP_TX_PRODUCER_COUNT 4
#define CDH_SLIP_TX_to_CDH_SLIP_RX 0
#define CDH_SLIP_TX_to_UHF_RX 1
#define Internal_Message_to_CDH_SLIP_TX 2

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

#pragma PERSISTENT(uhf_tx_muted)
bool uhf_tx_muted = false;

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
    CDH_SLIP_TX_producer_list[CDH_SLIP_TX_to_CDH_SLIP_RX].producer = &CDH_SLIP_RX; // TODO fix wording its backwards
    CDH_SLIP_TX_producer_list[CDH_SLIP_TX_to_CDH_SLIP_RX].packet_list_offset = 0;
    CDH_SLIP_TX_producer_list[CDH_SLIP_TX_to_UHF_RX].producer = &UHF_RX; // TODO fix wording its backwards
    CDH_SLIP_TX_producer_list[CDH_SLIP_TX_to_UHF_RX].packet_list_offset = 0;
    CDH_SLIP_TX_producer_list[Internal_Message_to_CDH_SLIP_TX].producer = &Internal_Message_Prod;
    CDH_SLIP_TX_producer_list[Internal_Message_to_CDH_SLIP_TX].packet_list_offset = 0;

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

#define FIFO_FILL 48

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

void CDH_SLIP_TX_consume_CDH_SLIP_RX(){

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
#ifdef FLIGHTCESIUM
        if(memcmp(data+offset+12, internal_ip, 4) != 0){ // Check that IP address not meant for internal

            // Starts sending packet through UART
            SLIP_start(&CDH_SLIP, cd.pinfo, cd.pcs->producer->buffer);

            cd.pinfo->state = BEING_CONSUMED;
            CDH_SLIP_TX.state = BUSY;

        }
#endif

#ifdef LAUNCHPAD
        if(memcmp(data+offset+12, internal_ip, 4) != 0 && data[offset + 1] != 0x69){ // Check that IP address not meant for internal

            // Starts sending packet through UART
            SLIP_start(&CDH_SLIP, cd.pinfo, cd.pcs->producer->buffer);

            cd.pinfo->state = BEING_CONSUMED;
            CDH_SLIP_TX.state = BUSY;

        }
#endif

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

    // TX disabled
    if(uhf_tx_muted) return;

    Consumer_Data cd = {0};

    while(consume(&UHF_TX, CDH_SLIP_RX_to_UHF_TX, &cd)){

        uint16_t offset = cd.pinfo->packet_offset;
        uint8_t * data = cd.pcs->producer->buffer;

#ifdef FLIGHTCESIUM
        if(memcmp(data+offset+12, internal_ip, 4) != 0){ // Check that IP address not meant for internal
#endif
#ifdef LAUNCHPAD
        if(1){
#endif

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

    // TX disabled
    if(uhf_tx_muted) return;

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

#ifdef FLIGHTCESIUM
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
                case COMMAND_BSL_PASS:
                    {
                        uint8_t route_cdh[1] = {0x03};
                        Internal_Message_produce_packet(route_cdh, 1, false);
                        Internal_Message_produce_packet((void*)0xFFE0, 0x20, true);
                    }
                    break;
                case COMMAND_CHANGE_BAUD:
                    UHF_change_baud(data[offset+29]);
                    break;
                default:
                    break;
            }

            cd.pinfo->state = CONSUMED;

        }
#endif

        advance_pcs(cd.pcs);

    }
}

// IPv4 checksum stuff
static inline uint16_t voidtouint16(void* mem, size_t offset) {
    return *(((uint16_t*)mem)+offset);
}

uint16_t csum(void* mem, size_t len) {
    uint32_t num = 0;
    size_t i = 0;
    while(i < (len-1)) {
        num += SWAP16(voidtouint16(mem, i/2));
        i += 2;
    }
    if (i < len) num += SWAP16(voidtouint16(mem, i/2));
    while ((num >> 0x10) != 0) {
        num = (num & 0xffff) + (num >> 0x10);
    }
    return ~(uint16_t)num;
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
                case COMMAND_MUTE_UHF_TX:
                    {
                        // Muting TX requires password
                        if(memcmp(data+offset+28+1,mute_passwd,32) == 0){
                            uhf_tx_muted = true;
                        }
                    }
                    break;
                case COMMAND_UNMUTE_UHF_TX:
                    {
                        uhf_tx_muted = false;
                    }
                    break;
                case COMMAND_CHANGE_BAUD:
                    UHF_change_baud(data[offset+29]);
                    break;

            }

            cd.pinfo->state = CONSUMED;

        }

#ifdef LAUNCHPAD
        if(data[offset + 1] == 0x69){
            uint8_t buffer[300];
            p_ip_header_t ip_hdr      = (p_ip_header_t)(buffer);
            p_udp_header_t udp_hdr    = (p_udp_header_t)(buffer + ETH_IP_HEADER_SIZE);

            int i;
            size_t packet_length = ETH_IP_HEADER_SIZE + ETH_UDP_HEADER_SIZE + cd.pinfo->packet_length -2;

            memset(buffer, 0, 300);

            ip_hdr->ip_hl_v = 5 | (4 << 4); // header then version this order from swapping
            ip_hdr->ip_tos = 0;
            ip_hdr->ip_len = SWAP16( (ETH_IP_HEADER_SIZE + ETH_UDP_HEADER_SIZE + cd.pinfo->packet_length -2) );
            ip_hdr->ip_ttl = 64;
            ip_hdr->ip_p = IP_PROT_UDP;
            ip_hdr->ip_off = 0x40; // don't fragment, magic value

            for (i = 0; i < 4; i++){
                ip_hdr->ip_src[i] = sat_ip[i];
                ip_hdr->ip_dst[i] = ground_ip[i];
            }
            udp_hdr->len = SWAP16( (ETH_UDP_HEADER_SIZE + cd.pinfo->packet_length -2) );
            udp_hdr->destport = SWAP16(35770ul);
            udp_hdr->srcport  = SWAP16(35770ul);
            udp_hdr->chksum = 0; //checksum is optional

            ip_hdr->ip_sum = csum((uint16_t *) ip_hdr, ETH_IP_HEADER_SIZE);
            ip_hdr->ip_sum = SWAP16(ip_hdr->ip_sum);

            memcpy((uint8_t*)udp_hdr + ETH_UDP_HEADER_SIZE, data + offset + 2, cd.pinfo->packet_length -2);

            Internal_Message_produce_packet(buffer, packet_length, true);

            cd.pinfo->state = CONSUMED;
        }
#endif

        advance_pcs(cd.pcs);

    }
}

CDH_SLIP_TX_consume_Internal_Message(){

    // SLIP TX not currently DMAing anything
    if(CDH_SLIP_TX.state != IDLE) return;

    Consumer_Data cd = {0};

    while(consume(&CDH_SLIP_TX, Internal_Message_to_CDH_SLIP_TX, &cd)){

        uint16_t offset = cd.pinfo->packet_offset;
        uint8_t * data = cd.pcs->producer->buffer;

        if(*(data+offset) == 3){ // Opcode for COM to CDH, using as TNC port number where EPS in port 3
            // Starts sending packet through UART
            SLIP_start(&CDH_SLIP, cd.pinfo, cd.pcs->producer->buffer);

            cd.pinfo->state = BEING_CONSUMED;
            CDH_SLIP_TX.state = BUSY;

        }

#ifdef LAUNCHPAD
        if(*(data+offset) == 0x45 && memcmp(data+offset+12, sat_ip, 4) == 0 && memcmp(data+offset+16, ground_ip, 4) == 0){
            SLIP_start(&CDH_SLIP, cd.pinfo, cd.pcs->producer->buffer);

            cd.pinfo->state = BEING_CONSUMED;
            CDH_SLIP_TX.state = BUSY;
        }
#endif

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

static inline void ACK(bool state){
    if(state){
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN1);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN1);
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

    CDH_SLIP_TX_consume_Internal_Message();

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

    // Check on hardware
    // TODO put somewhere else
    if(UHF_IRQ){
        SPIRIT1_get_irq_status_cb(&sconf, UHF_irq_cb);
        UHF_IRQ = false;
    } else {
        if(GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3) == 0){
            // More problems already...
            SPIRIT1_get_irq_status_cb(&sconf, UHF_irq_cb);
        }
    }

    // Do flow control
    {

        int pos_rx = CDH_SLIP_RX.packet_list_offset;
        int pos_tx = UHF_TX_producer_list[CDH_SLIP_RX_to_UHF_TX].packet_list_offset;

        int ahead = 0;

        if(pos_tx > pos_rx){
            ahead = pos_rx + (CDH_SLIP_RX_PACKET_LIST_SIZE-pos_tx);
        } else {
            ahead = pos_rx-pos_tx;
        }

        if(ahead > 16){ // MAGIC number 16 packets we will try to keep buffer at
            ACK(true); // packet buffer too full don't send ACK high
        } else {
            ACK(false); // packet buffer low keep sending ACK low
        }

    }

}
