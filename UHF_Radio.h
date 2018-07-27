#ifndef __UHF_RADIO_H
#define __UHF_RADIO_H

#include "SPIRIT1.h"

typedef enum {

    TX,
    RX,
    RX_PARTIAL,
    RX_DONE

} UHF_Radio_State;

extern UHF_Radio_State uhf_radio_state;

extern SPIRIT1_CONFIG sconf;

void UHF_Radio_init();

void UHF_init_RX();
void UHF_init_TX();

void UHF_TX_Test();

#endif
