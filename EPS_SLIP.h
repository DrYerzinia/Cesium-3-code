#ifndef __EPS_SLIP_H_
#define __EPS_SLIP_H_

#include "Producer.h"
#include "Consumer.h"

extern SLIP_Interface EPS_SLIP;

void EPS_SLIP_init(Consumer * EPS_SLIP_TX, Producer * EPS_SLIP_RX);

#endif
