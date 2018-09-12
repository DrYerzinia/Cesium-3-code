#ifndef __UHF_RADIO_H
#define __UHF_RADIO_H

#include "SPIRIT1.h"

typedef enum {

    TX,
    RX,
    RX_PARTIAL,
    RX_DONE

} UHF_Radio_State;

typedef enum {

    B4800,
    B9600,
    B19200,
    B38400,
    B76800,
    B115200

} UHF_Radio_Baud;

extern UHF_Radio_Baud uhf_radio_baud;
extern UHF_Radio_State uhf_radio_state;

extern SPIRIT1_CONFIG sconf;

void UHF_Radio_init();

void UHF_init_RX();
void UHF_init_TX();

void UHF_set_modulation_gfsk4k8();
void UHF_set_modulation_gfsk9k6();
void UHF_set_modulation_gfsk19k2();
void UHF_set_modulation_gfsk38k4();

void UHF_change_baud(UHF_Radio_Baud new_baud);

void UHF_TX_Test();

#endif
