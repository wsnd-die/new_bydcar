//
// Created by 35037 on 2026/9/13.
//

#include "msp_uart2.h"

// static struct
// {
//
// };

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        tx_finish_flag=1;
    }
}