#include <stdint.h>
#include <stdbool.h>

#include "driverlib.h"

#include "Packet_Manager.h"
#include "UHF_Radio.h"

#include "EPS_HnS.h"

void Init_GPIO(void);
void Init_Clock(void);
void Init_UHFSPI();
void Init_UART(void);

static inline void RED_LED(bool state){
    if(state){
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
}

static inline void WHITE_LED(bool state){
    if(state){
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1);
    }
}

static inline void GREEN_LED(bool state){
    if(state){
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN2);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN2);
    }
}

static inline void Init_SysTick(){

    //CCTL0 = CCIE;                             // CCR0 interrupt enabled
    //TACTL = TASSEL_2 + MC_1 + ID_3;           // SMCLK/8, upmode
    //CCR0 =  10000;                           // 12.5 Hz

    Timer_A_initUpModeParam ta0_param = {
      TIMER_A_CLOCKSOURCE_ACLK,
      TIMER_A_CLOCKSOURCE_DIVIDER_1,
      3277, // Timer Period 10Hz tick rate
      TIMER_A_TAIE_INTERRUPT_DISABLE,
      TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
      TIMER_A_DO_CLEAR,
      true
    };

    Timer_A_initUpMode(TA0_BASE, &ta0_param);

    //Timer_A_enableInterrupt(__MSP430_BASEADDRESS_T1A0__);

}

uint8_t heartbeat_counter = 0;

uint8_t ticks_uhf_tx_pc_same;
uint16_t last_uhf_tx_packet_counter;

uint8_t ticks_uhf_rx_partial = 0;

#define EPS_HNS_BEACON_RATE 15 // Beacon every 15 seconds
uint16_t EPS_HnS_beacon_time_counter = 0;

#define HIGHER_BAUD_TIMEOUT 15*60*10 // 15 minutes
uint16_t uhf_baud_timeout_counter  = 0;

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void){

    // If we are in a higher baud for more than 15 minutes reset our baud to 4k8
    if(uhf_radio_baud == B4800){
        uhf_baud_timeout_counter = 0;
    } else {
        uhf_baud_timeout_counter++;
        if(uhf_baud_timeout_counter == HIGHER_BAUD_TIMEOUT){
            UHF_set_modulation_gfsk4k8();
        }
    }

    if(EPS_HnS_beacon_count == -1 || EPS_HnS_beacon_count > 0){

        EPS_HnS_beacon_time_counter++;
        if(EPS_HnS_beacon_time_counter >= EPS_HNS_BEACON_RATE * 10){

            if(EPS_HnS_len > 0){

                uint8_t tnc_port = 2;
                Internal_Message_produce_packet(&tnc_port, sizeof(tnc_port), false);
                Internal_Message_produce_packet(EPS_HnS, EPS_HnS_len, true);

            }

            EPS_HnS_beacon_time_counter = 0;
            if(EPS_HnS_beacon_count > 0){
                EPS_HnS_beacon_count--;
            }

        }

    }

    heartbeat_counter++;
    if(heartbeat_counter == 10){
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        heartbeat_counter = 0;
    }

    if(uhf_radio_state == RX_PARTIAL){

        ticks_uhf_rx_partial++;

        if(ticks_uhf_rx_partial > 10){

            // RESET RX buffer to last starting p
            UHF_RX.write_location = UHF_RX.last_pkt_start_location;
            UHF_init_RX();

        }

    } else {

        ticks_uhf_rx_partial = 0;

    }

    if(uhf_radio_state == TX){

        // TODO TX watchdog
        if(last_uhf_tx_packet_counter == uhf_tx_packet_counter){

            ticks_uhf_tx_pc_same++;

            // If we are in TX but haven't TXed any new packets for 1 second we need to stop TX cause something is wrong.
            if(ticks_uhf_tx_pc_same > 10){
                UHF_TX.state = IDLE;
                UHF_init_RX();
            }

        } else {

            ticks_uhf_tx_pc_same = 0;
            last_uhf_tx_packet_counter = uhf_tx_packet_counter;

        }

        WHITE_LED(true);
        GREEN_LED(false);

    } else {

        WHITE_LED(false);
        GREEN_LED(true);

    }

}

int main(void)
{

    WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    Init_GPIO();
    Init_Clock();

    Init_SysTick();

    Packet_Manger_init();

    __delay_cycles(1000);

    UHF_Radio_init();

    while(1){

        Packet_Manger_process();

        /*
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'A');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'B');
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
        __delay_cycles(2000000);*/

    }

	return 0;
}

/*
 * GPIO Initialization
 */
void Init_GPIO()
{

    // Set all GPIO pins to output low for low power
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1|GPIO_PIN2|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7|GPIO_PIN8|GPIO_PIN9|GPIO_PIN10|GPIO_PIN11|GPIO_PIN12|GPIO_PIN13|GPIO_PIN14|GPIO_PIN15);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0); // PA Supply Off
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1); // Disable Digital Pot CS
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); // Disable SPIRIT1 CS
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN6); // Disable RS485 Driver
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4); // Disable SPIRIT1

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7|GPIO_PIN8|GPIO_PIN9|GPIO_PIN10|GPIO_PIN11|GPIO_PIN12|GPIO_PIN13|GPIO_PIN14|GPIO_PIN15);
    //GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN3);  // INT

    // Configure P2.0 - UCA0TXD and P2.1 - UCA0RXD
    //Set TXD low for low power mode
    //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    //GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);

    // EPS UART
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION);

    // Configure UHF SPI
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN2/*|GPIO_PIN3*/, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_PJ,
           GPIO_PIN4 + GPIO_PIN5,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

/*
 * Clock System Initialization
 */
void Init_Clock()
{
    // Set DCO frequency to 8 MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768, 0);
    //Set ACLK=LFXT
    CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //Start XT1 with no time out
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);
}


