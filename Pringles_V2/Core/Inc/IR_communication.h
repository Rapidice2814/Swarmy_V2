/*
 * IR_communication.h
 *
 *  Created on: Feb 17, 2025
 *      Author: Csongor
 */

#ifndef INC_IR_COMMUNICATION_H_
#define INC_IR_COMMUNICATION_H_

#include "main.h"

#define IR_DATA_SIZE 5       //Size of the actual IR data in bytes
#define IR_PACKET_SIZE (1 + IR_DATA_SIZE + 2 + 1)     //Size of the IR packet in bytes
#define IR_TX_CHANNEL_COUNT 4   //Number of TX IR channels
#define IR_RX_CHANNEL_COUNT 8   //Number of RX IR channels

typedef enum{
    IR_OK,
    IR_ERROR
}IR_StatusTypeDef;

typedef enum{
    IR_TX_IDLE,
    IR_TX_SENDING
}IR_TX_StateTypeDef;

typedef struct{
    uint8_t buffer[IR_TX_CHANNEL_COUNT][IR_DATA_SIZE];
    IR_TX_StateTypeDef state[IR_TX_CHANNEL_COUNT];
    uint8_t byte_counter;
    uint8_t bit_counter;
}IR_TX_HandleTypeDef;


IR_StatusTypeDef IR_TX_Init(IR_TX_HandleTypeDef *hir_tx);
IR_StatusTypeDef IR_TX_SetBuffer(IR_TX_HandleTypeDef *hir_tx, uint8_t channel, uint8_t *data);
IR_StatusTypeDef IR_TX_Start(IR_TX_HandleTypeDef *hir_tx, uint8_t channel);
IR_StatusTypeDef IR_TX_Stop(IR_TX_HandleTypeDef *hir_tx, uint8_t channel);
IR_StatusTypeDef IR_TX_Step(IR_TX_HandleTypeDef *hir_tx);

#endif /* INC_IR_COMMUNICATION_H_ */
