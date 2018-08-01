#ifndef __CDH_SLIP_H_
#define __CDH_SLIP_H_

#include "Producer.h"
#include "Consumer.h"

extern SLIP_Interface CDH_SLIP;

void CDH_SLIP_init(Consumer * CDH_SLIP_TX, Producer * CDH_SLIP_RX);

#endif
