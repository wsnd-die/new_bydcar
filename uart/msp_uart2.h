//
// Created by 35037 on 2026/9/13.
//

#ifndef BYDCAR_G491VET6_MSP_UART2_H
#define BYDCAR_G491VET6_MSP_UART2_H
#include "stm32g4xx_hal.h"

#define RX_BUF_LEN 128
uint8_t rx_buf[RX_BUF_LEN];
typedef enum {
    MSP_RX_WAIT_A3 = 0,
    MSP_RX_WAIT_BX,
    MSP_RX_COLLECT,
} msp_rx_state_t;

uint8_t tx_finish_flag=0;

#endif //BYDCAR_G491VET6_MSP_UART2_H