/*
 * IR_communication.c
 *
 *  Created on: Feb 17, 2025
 *      Author: Csongor
 */

#include "IR_communication.h"

static void increment_tx_counters(IR_TX_HandleTypeDef *hir_tx);
static uint8_t get_bit(uint8_t byte, uint8_t i);
static void set_tx_led(uint8_t channel, uint8_t state);


IR_StatusTypeDef IR_TX_Init(IR_TX_HandleTypeDef *hir_tx){
    for(int i = 0; i < IR_TX_CHANNEL_COUNT; i++){
        hir_tx->state[i] = IR_TX_IDLE;
    }

    for(int i = 0; i < IR_TX_CHANNEL_COUNT; i++){
        for(int j = 0; j < IR_DATA_SIZE; j++){
            hir_tx->buffer[i][j] = 0;
        }
    }

    hir_tx->byte_counter = 0;
    hir_tx->bit_counter = 0;

    return IR_OK;
}

IR_StatusTypeDef IR_TX_SetBuffer(IR_TX_HandleTypeDef *hir_tx, uint8_t channel, uint8_t *data){
    if(channel < IR_TX_CHANNEL_COUNT){
        // for(int i = 0; i < IR_DATA_SIZE; i++){
        //     hir_tx->buffer[channel][i] = data[i];
        // }
        memcpy(hir_tx->buffer[channel], data, IR_DATA_SIZE);
        return IR_OK;
    }
    return IR_ERROR;
}

IR_StatusTypeDef IR_TX_Start(IR_TX_HandleTypeDef *hir_tx, uint8_t channel){
    if(channel < IR_TX_CHANNEL_COUNT){
        hir_tx->state[channel] = IR_TX_SENDING;
        return IR_OK;
    }
    return IR_ERROR;
}

IR_StatusTypeDef IR_TX_Stop(IR_TX_HandleTypeDef *hir_tx, uint8_t channel){
    if(channel < IR_TX_CHANNEL_COUNT){
        hir_tx->state[channel] = IR_TX_IDLE;
        return IR_OK;
    }
    return IR_ERROR;
}



IR_StatusTypeDef IR_TX_Step(IR_TX_HandleTypeDef *hir_tx){
    for(int channel = 0; channel < IR_TX_CHANNEL_COUNT; channel++){
        if(hir_tx->state[channel] == IR_TX_SENDING){
            set_tx_led(channel, get_bit(hir_tx->buffer[channel][hir_tx->byte_counter], hir_tx->bit_counter)); 
        }
        else{
            set_tx_led(channel, 0);
        }
    }
    
    increment_tx_counters(hir_tx);


    return IR_OK;
}

static uint8_t get_bit(uint8_t byte, uint8_t i) {
    if (i < 8) {
        return (byte >> i) & 0x01;
    }
    return 0;
}

static void increment_tx_counters(IR_TX_HandleTypeDef *hir_tx){
    hir_tx->bit_counter++;
    if(hir_tx->bit_counter >= 8){
        hir_tx->bit_counter = 0;
        hir_tx->byte_counter++;
        if(hir_tx->byte_counter >= IR_PACKET_SIZE){
            hir_tx->byte_counter = 0;
        }
    }
}

static void set_tx_led(uint8_t channel, uint8_t state){
    switch(channel){
        case 0:
            HAL_GPIO_WritePin(IR1_GPIO_Port, IR1_Pin, state);
            break;
        case 1:
            HAL_GPIO_WritePin(IR2_GPIO_Port, IR2_Pin, state);
            break;
        case 2:
            HAL_GPIO_WritePin(IR3_GPIO_Port, IR3_Pin, state);
            break;
        case 3:
            HAL_GPIO_WritePin(IR4_GPIO_Port, IR4_Pin, state);
            break;
    }
}