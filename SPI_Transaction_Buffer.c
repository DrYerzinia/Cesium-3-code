#include "SPI_Transaction_Buffer.h"

#include <stddef.h>

void SPI_Transaction_Buffer_init(
        SPI_Transaction_Buffer * buf,
        SPI_Transaction * transactions,
        uint8_t * transaction_tx_data,
        uint8_t * transaction_rx_data,
        uint8_t max_transactions,
        uint16_t max_data){

    buf->transactions = transactions;
    buf->transaction_tx_data = transaction_tx_data;
    buf->transaction_rx_data = transaction_rx_data;

    buf->transaction_len = max_transactions;
    buf->transaction_data_len =  max_data;

    buf->transaction_idx_start = 0;
    buf->transaction_data_idx_start = 0;
    buf->transaction_idx_end = 0;
    buf->transaction_data_idx_end = 0;

}

void SPI_Transaction_Buffer_add(
        SPI_Transaction_Buffer * buf,
        SPI_Transaction_Callback callback,
        uint8_t cs,
        uint16_t len,
        uint8_t * data){

    // Insert next transaction
    SPI_Transaction * transaction = &(buf->transactions[buf->transaction_idx_end++]);
    if(buf->transaction_idx_end >= buf->transaction_len){
        buf->transaction_idx_end = 0;
    }

    transaction->callback = callback;
    transaction->data_len = len;
    transaction->cs = cs;

    // Copy transaction data to circular buffer
    if(buf->transaction_data_idx_end + transaction->data_len > buf->transaction_data_len){
        buf->transaction_data_idx_end = 0;
    }
    transaction->data_start = buf->transaction_data_idx_end;
    memcpy(&(buf->transaction_tx_data[buf->transaction_data_idx_end]), data, transaction->data_len);
    buf->transaction_data_idx_end += transaction->data_len;

}

SPI_Transaction * SPI_Transaction_Buffer_next(SPI_Transaction_Buffer * buf){

    if(buf->transaction_idx_start == buf->transaction_idx_end) return NULL;

    SPI_Transaction * transaction = &(buf->transactions[buf->transaction_idx_start++]);
    if(buf->transaction_idx_start >= buf->transaction_len){
        buf->transaction_idx_start = 0;
    }

    return transaction;

}
