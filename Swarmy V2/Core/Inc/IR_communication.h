/*
 * IR_communication.h
 *
 *  Created on: Feb 17, 2025
 *      Author: Csongor
 */

#ifndef INC_IR_COMMUNICATION_H_
#define INC_IR_COMMUNICATION_H_

#include "main.h"

#define IR_PACKAGE_SIZE 5       //Size of the IR package in bytes
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
    uint8_t buffer[IR_TX_CHANNEL_COUNT][IR_PACKAGE_SIZE];
    IR_TX_StateTypeDef state;
}IR_TX_HandleTypeDef;

IR_StatusTypeDef IR_TX_Init(IR_TX_HandleTypeDef *hir_tx);

#endif /* INC_IR_COMMUNICATION_H_ */
