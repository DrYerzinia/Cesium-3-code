#ifndef __SPI_TRANSACTION_BUFFER_H
#define __SPI_TRANSACTION_BUFFER_H

#include <stdint.h>

typedef void (*SPI_Transaction_Callback)(uint8_t *data, uint16_t len);

typedef struct {

    SPI_Transaction_Callback callback;

    uint16_t data_start;
    uint16_t data_len;

    uint8_t cs;

} SPI_Transaction;

typedef struct {

    SPI_Transaction * transactions;
    uint8_t * transaction_tx_data;
    uint8_t * transaction_rx_data;

    uint8_t transaction_len;
    uint16_t transaction_data_len;

    uint8_t transaction_idx_start;
    uint16_t transaction_data_idx_start;
    uint8_t transaction_idx_end;
    uint16_t transaction_data_idx_end;

} SPI_Transaction_Buffer;


void SPI_Transaction_Buffer_init(
        SPI_Transaction_Buffer * buf,
        SPI_Transaction * transactions,
        uint8_t * transaction_tx_data,
        uint8_t * transaction_rx_data,
        uint8_t max_transactions,
        uint16_t max_data);

void SPI_Transaction_Buffer_add(
        SPI_Transaction_Buffer * buf,
        SPI_Transaction_Callback callback,
        uint8_t cs,
        uint16_t len,
        uint8_t * data);

SPI_Transaction * SPI_Transaction_Buffer_next(SPI_Transaction_Buffer * buf);

#endif
