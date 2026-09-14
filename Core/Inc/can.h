/**
  ******************************************************************************
  * @file    can.h
  * @brief   Emm_V5 stepper bus (FDCAN2) interface.
  *
  *          The implementation lives in Core/Src/can.c. hardware/emm_5v.h and
  *          hardware/Common_used.h both include this header, so it is the
  *          shared declaration point for the CAN command/reply API.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Filled by HAL_FDCAN_RxFifo0Callback() in Core/Src/can.c. */
extern volatile uint32_t      can_error_step;
extern volatile uint32_t      can_error_code;
extern volatile uint32_t      can_error_count;
extern volatile uint8_t       can_rx_flag;
extern FDCAN_RxHeaderTypeDef  can_rx_header;
extern uint8_t                can_rx_data[8];

/* Exported functions prototypes ---------------------------------------------*/

/** @brief  Send one raw command frame on FDCAN2.
  * @param  cmd  Payload bytes to send.
  * @param  len  Number of bytes in @p cmd.
  * @retval 1 on success, 0 if the TX FIFO stayed full past the internal timeout.
  */
uint8_t can_SendCmd(__IO uint8_t *cmd, uint8_t len);

/** @brief  Query an Emm_V5 driver's status register (command 0x0F).
  * @retval 1 on success, 0 on timeout. */
uint8_t Emm_V5_Read_Status(uint8_t id, uint8_t *status, uint32_t timeout_ms);

/** @brief  Check whether motor @p id has finished its last move.
  * @retval 1 when the driver reports "reached", 0 otherwise. */
uint8_t Emm_V5_Is_Reached(uint8_t id);

/** @brief  Start the FDCAN2 receive path (filters + notification). */
void fdcan2_UserInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
