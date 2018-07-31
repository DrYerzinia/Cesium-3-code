#include "UHF_Radio.h"
#include "driverlib.h"
#include "SPI_Transaction_Buffer.h"
#include "Packet_Manager.h"

#include <stddef.h>

// UHF SPI Transaction Buffer
#define SPI_TRANSACTION_COUNT 20
#define SPI_DATA_LEN 200

SPI_Transaction_Buffer spi_trans_buf;
SPI_Transaction spi_transactions[SPI_TRANSACTION_COUNT];
uint8_t spi_tx_data[SPI_DATA_LEN];
uint8_t spi_rx_data[SPI_DATA_LEN];

#define TEMP_SPI_TRANSACTION_LEN 100

uint8_t spi_transaction_buf_pos;
uint8_t spi_transaction_buf[TEMP_SPI_TRANSACTION_LEN];

UHF_Radio_State uhf_radio_state = RX;

bool consuming_transactions  = false;

void Consume_Transactions(){

    if(consuming_transactions) return;

    consuming_transactions = true;

    while(1){

        SPI_Transaction * transact = SPI_Transaction_Buffer_next(&spi_trans_buf);
        if(transact == NULL){
            consuming_transactions = false;
            return;
        }

        switch(transact->cs){
            case 0:
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
                break;
            case 1:
                GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
                break;
        }

        int i;
        for(i = 0; i < transact->data_len; i++){

            EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, spi_trans_buf.transaction_tx_data[transact->data_start+i]);
            while(EUSCI_B_SPI_isBusy(EUSCI_B1_BASE) == EUSCI_B_SPI_BUSY);
            spi_trans_buf.transaction_rx_data[transact->data_start+i] = EUSCI_B_SPI_receiveData(EUSCI_B1_BASE);

        }

        switch(transact->cs){
            case 0:
                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
                break;
            case 1:
                GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
                break;
        }

        if(transact->callback != NULL){
            transact->callback(spi_trans_buf.transaction_rx_data+transact->data_start, transact->data_len);
        }
    }

}

void SPIRIT1_SPI_start(){

    spi_transaction_buf_pos = 0;
    //GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);

}

void SPIRIT1_SPI_stop(data_callback callback){

    SPI_Transaction_Buffer_add(&spi_trans_buf, callback, 0, spi_transaction_buf_pos, spi_transaction_buf);
    //GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);

    Consume_Transactions();

}

uint8_t SPIRIT1_SPI_transfer(uint8_t data){

    spi_transaction_buf[spi_transaction_buf_pos++] = data;
    return 0;
    /*
    EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, data);
    while(EUSCI_B_SPI_isBusy(EUSCI_B1_BASE) == EUSCI_B_SPI_BUSY);
    return EUSCI_B_SPI_receiveData(EUSCI_B1_BASE);*/

}

void Digital_Pot_SPI_start(){
    spi_transaction_buf_pos = 0;
    //GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
}
void Digital_Pot_SPI_stop(){
    SPI_Transaction_Buffer_add(&spi_trans_buf, NULL, 1, spi_transaction_buf_pos, spi_transaction_buf);
    //GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
}

uint8_t Digital_Pot_SPI_transfer(uint8_t data){
    spi_transaction_buf[spi_transaction_buf_pos++] = data;
    return 0;
    /*
    EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, data);
    while(EUSCI_B_SPI_isBusy(EUSCI_B1_BASE) == EUSCI_B_SPI_BUSY);
    return EUSCI_B_SPI_receiveData(EUSCI_B1_BASE);*/
}

static inline void UHF_PA_Gate_EN(bool state){
    if(state){
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    }
}

static inline void UHF_PA_Power_EN(bool state){
    if(state){
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
    } else {
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
    }
}

static inline void UHF_TX_nRX(bool state){
    if(state){
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
    }
}

static inline void UHF_LNA_EN(bool state){
    if(state){
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4);
    }
}

static inline void UHF_SDN(bool state){
    if(state){
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
    }
}

SPIRIT1_CONFIG sconf = {0};

void UHF_rx_fifo_cb(uint8_t *data, uint16_t len){

    bool finish = false;

    if(uhf_radio_state == RX_DONE){
        uhf_radio_state = RX;
        finish = true;
    }
    UHF_RX_produce_packet(data+2, len-2, finish);

}

void UHF_rx_fifo_len(uint8_t *data, uint16_t len){

    uint8_t bytes_available = data[2];
    read_block_cb(&sconf, SPIRIT1_FIFO, bytes_available, UHF_rx_fifo_cb);

}

void UHF_irq_cb(uint8_t *data, uint16_t len){

  volatile uint32_t val = SPIRIT1_get_irq_status_from_data(data + 2);
  switch(uhf_radio_state){
      case TX:
          // If the TX FIFO is almost empty send more data
          if(val & SPIRIT1_IRQ_TX_FIFO_ALMOST_EMPTY){

              UHF_TX_consume_data();

          } else if(val & SPIRIT1_IRQ_TX_DATA_SENT){
              // If the TX data was sent, check to see if UHF TX packet manager is idle
              // if it is switch back to receive
              if(UHF_TX.state == IDLE){
                  UHF_init_RX();
              }

          }
          break;
      case RX:
      case RX_PARTIAL:
          if(val & SPIRIT1_IRQ_RX_DATA_READY){

              uhf_radio_state = RX_DONE;
              SPIRIT1_get_rx_fifo_len_cb(&sconf, UHF_rx_fifo_len);
              uhf_rx_packet_counter++;

          } else if(val & SPIRIT1_IRQ_RX_FIFO_ALMOST_FULL){

              uhf_radio_state = RX_PARTIAL;
              SPIRIT1_get_rx_fifo_len_cb(&sconf, UHF_rx_fifo_len);

          }
          // if RX FIFO almost full or RX Data Ready
          // Read FIFO
          break;
  }

}

void UHF_default_config(){

    UHF_SDN(false); // Turn on Spirit 1

    __delay_cycles(100000);

    // Configure PA
    Digital_Pot_SPI_start();
    Digital_Pot_SPI_transfer(130);
    Digital_Pot_SPI_stop();

    // Configure Radio
    SPIRIT1_disable_smps(&sconf);
    //SPIRIT1_set_crystal_correction(&sconf, -85);
    SPIRIT1_configure_gpio(&sconf, 0, SPIRIT1_GPIO_nIRQ | SPIRIT1_GPIO_DIG_OUT_LOWPWR);
    SPIRIT1_configure_irq_mask(&sconf, SPIRIT1_IRQ_RX_DATA_READY | SPIRIT1_IRQ_RX_FIFO_ALMOST_FULL | SPIRIT1_IRQ_TX_FIFO_ALMOST_EMPTY | SPIRIT1_IRQ_TX_DATA_SENT);
    SPIRIT1_setup_FIFO_thresholds(&sconf, 0x30, 0x30, 0x30, 20);
    SPIRIT1_configure_pa(&sconf, 1); // 11dBm but with boost config should be about 16dBm

    // Configure IF Offset for 50MHz crystal
    SPIRIT1_set_ifoffset(&sconf, 0x36, 0xAC);

    SPIRIT1_PACKET_CONFIG pkt_conf;
    pkt_conf.pkt_type = BASIC;
    pkt_conf.addr_len = 0;
    pkt_conf.ctrl_len = 0;
    pkt_conf.preamble_len = 3;
    pkt_conf.sync_len = 3;
    pkt_conf.var_len = true;
    pkt_conf.len_wid = 8;      // up to 256 byte packet size
    pkt_conf.crc_mode = CRC07;
    pkt_conf.whitening_en = true;
    pkt_conf.fec_en = false;
    pkt_conf.crc_check_en = true;
    SPIRIT1_set_packet_config(&sconf, &pkt_conf);

    SPIRIT1_FREQUENCY_CONFIG freq_conf;
    freq_conf.synt = 12626952;           // 401.4MHz
    freq_conf.band = SPIRIT1_12_BAND;    // 433MHz band
    freq_conf.refdiv = SPIRIT1_REFDIV_1; // Don't divide ref
    freq_conf.wcp = 2;
    freq_conf.vco_hl = SPIRIT1_VCO_L_SEL;
    SPIRIT1_set_base_frequency(&sconf, &freq_conf);

    SPIRIT1_MODULATION_CONFIG gfsk_4_8_kbps;
    gfsk_4_8_kbps.datarate_m = 0x93;   // 4.8kbps
    gfsk_4_8_kbps.datarate_e = 0x07;
    gfsk_4_8_kbps.fdev_m  = 0x05;      // 2.4kHz deviation
    gfsk_4_8_kbps.fdev_e = 0x01;
    gfsk_4_8_kbps.chflt_m  = 0x01;     // 26kHz Channel Filter
    gfsk_4_8_kbps.chflt_e = 0x05;
    gfsk_4_8_kbps.cw = 0;              // No CW tone
    gfsk_4_8_kbps.bt_sel = BT_SEL_1;
    gfsk_4_8_kbps.mod_type = GFSK;
    SPIRIT1_set_modulation(&sconf, &gfsk_4_8_kbps);

    /*
    SPIRIT1_MODULATION_CONFIG gmsk_4_8_kbps;
    gmsk_4_8_kbps.datarate_m = 0x93;   // 4.8kbps
    gmsk_4_8_kbps.datarate_e = 0x07;
    gmsk_4_8_kbps.fdev_m  = 0x05;      // 1.2kHz deviation
    gmsk_4_8_kbps.fdev_e = 0x00;
    gmsk_4_8_kbps.chflt_m  = 0x01;     // 26kHz Channel Filter
    gmsk_4_8_kbps.chflt_e = 0x05;
    gmsk_4_8_kbps.cw = 0;              // No CW tone
    gmsk_4_8_kbps.bt_sel = BT_SEL_0_5;
    gmsk_4_8_kbps.mod_type = GFSK;
    SPIRIT1_set_modulation(&sconf, &gmsk_4_8_kbps);
    */
    /*
    SPIRIT1_MODULATION_CONFIG gmsk_9_6_kbps;
    gmsk_9_6_kbps.datarate_m = 0x8B;   // 9.6kbps
    gmsk_9_6_kbps.datarate_e = 0x08;
    gmsk_9_6_kbps.fdev_m  = 0x04;      // 2.4kHz deviation
    gmsk_9_6_kbps.fdev_e = 0x01;
    gmsk_9_6_kbps.chflt_m  = 0x01;     // 26kHz Channel Filter
    gmsk_9_6_kbps.chflt_e = 0x05;
    gmsk_9_6_kbps.cw = 0;              // No CW tone
    gmsk_9_6_kbps.bt_sel = BT_SEL_0_5;
    gmsk_9_6_kbps.mod_type = GFSK;
    SPIRIT1_set_modulation(&sconf, &gmsk_9_6_kbps);
    */
    /*
    SPIRIT1_MODULATION_CONFIG gmsk_38_4_kbps;
    gmsk_38_4_kbps.datarate_m = 131; // 38.4kbps
    gmsk_38_4_kbps.datarate_e = 10;
    gmsk_38_4_kbps.fdev_m  = 35;      // 9.8kHz deviation
    gmsk_38_4_kbps.fdev_e = 3;
    gmsk_38_4_kbps.chflt_m  = 0;     // 54kHz Channel Filter
    gmsk_38_4_kbps.chflt_e = 4;
    gmsk_38_4_kbps.cw = 0;           // No CW tone
    gmsk_38_4_kbps.bt_sel = BT_SEL_0_5;
    gmsk_38_4_kbps.mod_type = GFSK;
    SPIRIT1_set_modulation(&sconf, &gmsk_38_4_kbps);
    */
    /*
    SPIRIT1_MODULATION_CONFIG gfsk_38_4_kbps;
    gfsk_38_4_kbps.datarate_m = 131; // 38.4kbps
    gfsk_38_4_kbps.datarate_e = 10;
    gfsk_38_4_kbps.fdev_m  = 5;      // 20kHz deviation ~
    gfsk_38_4_kbps.fdev_e = 4;
    gfsk_38_4_kbps.chflt_m  = 1;     // 100kHz Channel Filter
    gfsk_38_4_kbps.chflt_e = 3;
    gfsk_38_4_kbps.cw = 0;           // No CW tone
    gfsk_38_4_kbps.bt_sel = BT_SEL_1;
    gfsk_38_4_kbps.mod_type = GFSK;
    SPIRIT1_set_modulation(&sconf, &gfsk_38_4_kbps);
    */


    // Configure IRQ sources

    // enable SPIRIT1 interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P4, GPIO_PIN3, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN3);

}

void UHF_RX_Test(){

    UHF_default_config();

    // Turn off PA
    UHF_PA_Gate_EN(false);
    UHF_PA_Power_EN(false);

    // RF switch to RX
    UHF_TX_nRX(false);

    // Turn on LNA
    UHF_LNA_EN(true);

    SPIRIT1_enable_persistent_rx(&sconf);
    SPIRIT1_enable_rx(&sconf);

    SPIRIT1_get_irq_status_cb(&sconf, UHF_irq_cb);

    uhf_radio_state = RX;

    /*
    while(1){

        SPIRIT1_get_irq_status_cb(&sconf, UHF_irq_cb);

        volatile uint32_t irq_stat = SPIRIT1_get_irq_status(&sconf);
        volatile uint8_t rx_fifo_len = SPIRIT1_get_rx_fifo_len(&sconf);

        if((irq_stat & SPIRIT1_IRQ_RX_DATA_READY) != 0){ // rx_fifo_len >= 12){
            uint8_t data[SPIRIT1_FIFO_SIZE];
            uint8_t bytes_rxed = SPIRIT1_rxfifo_dump(&sconf, data, SPIRIT1_FIFO_SIZE);
            __delay_cycles(8000000);
        }

        __delay_cycles(8000000);

    }*/

}

void UHF_init_RX(){

    SPIRIT1_disable_persistent_tx(&sconf);
    SPIRIT1_abort(&sconf);

    // Flush TX Fifo incase some crap got left in it
    SPIRIT1_flush_tx(&sconf);

    // Turn off PA
    UHF_PA_Gate_EN(false);
    UHF_PA_Power_EN(false);

    // RF switch to RX
    UHF_TX_nRX(false);

    // Turn on LNA
    UHF_LNA_EN(true);

    SPIRIT1_enable_persistent_rx(&sconf);
    SPIRIT1_enable_rx(&sconf);

    SPIRIT1_get_irq_status_cb(&sconf, UHF_irq_cb);

    uhf_radio_state = RX;

}

void UHF_init_TX(){

    SPIRIT1_disable_persistent_tx(&sconf);
    SPIRIT1_abort(&sconf);

    // Turn off LNA
    UHF_LNA_EN(false);

    // RF switch to TX
    UHF_TX_nRX(true);

    // Turn on PA
    UHF_PA_Power_EN(true);
    UHF_PA_Gate_EN(true);  // TODO maybe sync with packet start
    // TODO watchdog to make sure we don't get stuck with PA burning power

    __delay_cycles(4000000); // TODO timer callback for PA warmup

    SPIRIT1_get_irq_status_cb(&sconf, UHF_irq_cb);

    SPIRIT1_enable_persistent_tx(&sconf);
    SPIRIT1_enable_tx(&sconf);

    uhf_radio_state = TX;

}

void UHF_TX_Test(){

    UHF_default_config();

    while(1){

        // RF switch to TX
        UHF_TX_nRX(true);

        // Turn on PA
        UHF_PA_Power_EN(true);
        __delay_cycles(10000);
        UHF_PA_Gate_EN(true);

        __delay_cycles(1000000);

        // TRANSMIT
        SPIRIT1_transmit_packet(&sconf, (uint8_t*)"Hello world!Hello world!Hello world!Hello world!Hello world!Hello world!Hello world!", 84);
        //SPIRIT1_lock_tx(&sconf);

        __delay_cycles(8000000);

        //volatile uint32_t irq_stat = SPIRIT1_get_irq_status(&sconf);

        // Turn off PA
        UHF_PA_Gate_EN(false);
        UHF_PA_Power_EN(false);

        // RF switch to RX
        UHF_TX_nRX(false);

        __delay_cycles(5*8000000);

    }
}

void Init_UHFSPI(){

    EUSCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = 8000000;
    param.desiredSpiClock = 1000000;
    param.msbFirst = EUSCI_B_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    param.spiMode = EUSCI_B_SPI_3PIN;//EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW;

    EUSCI_B_SPI_initMaster(EUSCI_B1_BASE, &param);
    EUSCI_B_SPI_enable(EUSCI_B1_BASE);

}

void UHF_Radio_init(){

    SPI_Transaction_Buffer_init(&spi_trans_buf, spi_transactions, spi_tx_data, spi_rx_data, SPI_TRANSACTION_COUNT, SPI_DATA_LEN);

    Init_UHFSPI();

    sconf.SPI_start = SPIRIT1_SPI_start;
    sconf.SPI_stop = SPIRIT1_SPI_stop;
    sconf.SPI_transfer = SPIRIT1_SPI_transfer;

    SPIRIT1_init(&sconf);

    UHF_default_config();
    UHF_init_RX();

}

// SPI Device for UHF Radio
#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void){

    switch (__even_in_range(UCB1IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG: break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
    }
}

#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void){

    uint16_t stat = GPIO_getInterruptStatus(GPIO_PORT_P4, GPIO_PIN3);

    if((stat & GPIO_PIN3)){

        SPIRIT1_get_irq_status_cb(&sconf, UHF_irq_cb);
        GPIO_clearInterrupt(GPIO_PORT_P4, GPIO_PIN3);

    }
}
