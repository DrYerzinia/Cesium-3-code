
#include "driverlib.h"

#include "SLIP_Interface.h"

SLIP_Interface CDH_SLIP;

void CDH_slip_write(uint8_t data){

    HWREG16(EUSCI_A0_BASE + OFS_UCAxTXBUF) = data;

}

void CDH_SLIP_init_UART(){

    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 4;
    param.firstModReg = 5;
    param.secondModReg = 0x55;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
        return;

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE,
                                EUSCI_A_UART_RECEIVE_INTERRUPT);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE,
                                EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT);

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
                                 EUSCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt

    // Enable USCI_A0 TX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
                                 EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT); // Enable interrupt

    // Enable global interrupt
    __enable_interrupt();

}


void CDH_SLIP_init(Consumer * CDH_SLIP_TX, Producer * CDH_SLIP_RX){

    CDH_SLIP_init_UART();

    SLIP_init(&CDH_SLIP, CDH_slip_write, CDH_SLIP_TX, CDH_SLIP_RX);

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
