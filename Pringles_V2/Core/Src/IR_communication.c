/*
 * IR_communication.c
 *
 *  Created on: Feb 17, 2025
 *      Author: Csongor
 */

#include "IR_communication.h"


IR_StatusTypeDef IR_TX_Init(IR_TX_HandleTypeDef *hir_tx){
    hir_tx->state = IR_TX_IDLE;

    for(int i = 0; i < IR_TX_CHANNEL_COUNT; i++){
        for(int j = 0; j < IR_PACKAGE_SIZE; j++){
            hir_tx->buffer[i][j] = 0;
        }
    }

    return IR_OK;
}