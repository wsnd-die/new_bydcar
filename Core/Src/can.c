#include "../../hardware/Common_used.h"

extern FDCAN_HandleTypeDef hfdcan2;

#define FDCAN_TX_TIMEOUT_MS   20

volatile uint32_t can_error_step  = 0;
volatile uint32_t can_error_code  = 0;
volatile uint32_t can_error_count = 0;
volatile uint8_t  can_rx_flag     = 0;

/* ---- 总线物理状态快照, 给调试器看的 ----
 *
 * 之前的 can_error_code 装的是 hfdcan->ErrorCode, 那是 **IR 寄存器**的位,
 * 读不出总线到底怎么了。下面三个才是判断"能不能通"的关键:
 *
 *   can_psr        原始 PSR 寄存器:
 *                    bit2:0 LEC  最近错误码, 3 = ACK 错误(发了没人应答)
 *                    bit4:3 ACT  0=同步中(看不到总线活动) 1=空闲 2=接收 3=发送
 *                    bit5   EP   错误被动
 *                    bit6   EW   错误警告
 *                    bit7   BO   **总线关闭**
 *   can_psr_tec    发送错误计数, 没人 ACK 时每失败一帧 +8, 涨到 256 触发 bus-off
 *   can_busoff_cnt 进入 bus-off 的次数 (恢复代码生效前, 这里涨了就说明卡死) */
volatile uint32_t can_psr         = 0;
volatile uint32_t can_psr_tec     = 0;
volatile uint32_t can_busoff_cnt  = 0;

FDCAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[8];

static uint32_t fdcan_dlc_from_len(uint8_t len)
{
    switch (len)
    {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        default: return FDCAN_DLC_BYTES_8;
    }
}

static uint8_t FDCAN_WaitFreeTxFifo(uint32_t timeout_ms)
{
    uint32_t tickstart = HAL_GetTick();

    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0)
    {
        if ((HAL_GetTick() - tickstart) > timeout_ms)
        {
            return 0;
        }
        osDelay(1);
    }
    return 1;
}

uint8_t can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t k = 0;
    uint8_t l = 0;
    uint8_t packNum = 0;

    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];

    if (cmd == NULL)
        return 0;

    if (len < 2)
        return 0;

    j = len - 2;

    while (i < j)
    {
        k = j - i;

        txHeader.Identifier = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
        txHeader.IdType = FDCAN_EXTENDED_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;

        txData[0] = cmd[1];

        if (k < 8)
        {
            for (l = 0; l < k; l++, i++)
            {
                txData[l + 1] = cmd[i + 2];
            }

            txHeader.DataLength = fdcan_dlc_from_len(k + 1);
        }
        else
        {
            for (l = 0; l < 7; l++, i++)
            {
                txData[l + 1] = cmd[i + 2];
            }

            txHeader.DataLength = FDCAN_DLC_BYTES_8;
        }

        if (FDCAN_WaitFreeTxFifo(FDCAN_TX_TIMEOUT_MS) == 0)
        {
            can_error_step = 1;
            can_error_code = HAL_FDCAN_GetError(&hfdcan2);
            can_error_count++;
            printf("CANfail addr=0x%02X step=1 cnt=%lu\r\n", cmd[0], (unsigned long)can_error_count);
            return 0;
        }

        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData) != HAL_OK)
        {
            can_error_step = 2;
            can_error_code = HAL_FDCAN_GetError(&hfdcan2);
            can_error_count++;
            printf("CANfail addr=0x%02X step=2 cnt=%lu\r\n", cmd[0], (unsigned long)can_error_count);
            return 0;
        }

        packNum++;
    }

    can_error_step = 0;
    return 1;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == FDCAN2)
    {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
        {
            while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
            {
                if (HAL_FDCAN_GetRxMessage(
                        hfdcan,
                        FDCAN_RX_FIFO0,
                        &can_rx_header,
                        can_rx_data) == HAL_OK)
                {
                    can_rx_flag = 1;
                }
            }
        }
    }
}

uint8_t Emm_V5_Read_Status(uint8_t id, uint8_t *status, uint32_t timeout_ms)
{
    uint8_t cmd[3] = {id, 0x3A, 0x6B};
    uint32_t start;

    if (status == NULL)
        return 0;

    can_rx_flag = 0;

    if (can_SendCmd(cmd, 3) == 0)
        return 0;

    start = osKernelGetTickCount();

    while ((osKernelGetTickCount() - start) < timeout_ms)
    {
        if (can_rx_flag)
        {
            can_rx_flag = 0;

            uint8_t rx_id = (uint8_t)(can_rx_header.Identifier >> 8);

            /* 与位置读取同规律: [命令0x3A][0x01][状态][校验0x6B]
             * status = data[2], 校验 = data[3] */
            if (can_rx_header.IdType == FDCAN_EXTENDED_ID &&
                rx_id == id &&
                can_rx_data[0] == 0x3A &&
                can_rx_data[3] == 0x6B)
            {
                *status = can_rx_data[2];
                return 1;
            }
        }

        osDelay(1);
    }

    return 0;
}

uint8_t Emm_V5_Is_Reached(uint8_t id)
{
    uint8_t status = 0;

    if (Emm_V5_Read_Status(id, &status, 50) == 0)
        return 0;

    return (status & 0x02) ? 1 : 0;
}

void fdcan2_UserInit(void)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    /* ������չ֡������������ȫ����չ ID �� FIFO0 */
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * ȫ�ֹ�����
     * ��׼֡���ܾ�
     * ��չ֡���� FIFO0
     * Զ��֡���ܾ�
     */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                     FDCAN_REJECT,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* ���� FDCAN */
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    /* ���� FIFO0 ����Ϣ�ж� */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF,
                                       0) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================
 * FDCAN 错误回调 — bus-off 检测与恢复
 * 底盘/丝杆命令走 CAN, 一旦 bus-off 所有命令静默失败(电机不动),
 * 而舵机是 TIM PWM 不受影响 → 表现为"舵机动、电机不动、卡死"。
 * 这里检测 bus-off 并重新初始化 FDCAN 恢复通信。
 * ============================================================ */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t psr;

    can_error_count++;

    /* 先把总线状态原样抓下来, 这是判断"能不能通"的唯一可靠依据。 */
    psr = hfdcan->Instance->PSR;
    can_psr     = psr;
    can_psr_tec = (hfdcan->Instance->ECR & FDCAN_ECR_TEC);
    can_error_code = psr;

    /* ------------------------------------------------------------------
     * 下面这段曾经是**死代码**, 从来不会执行。
     *
     * 原来写的是 `err & FDCAN_PSR_BO`, 其中 err = HAL_FDCAN_GetError(hfdcan)
     * 返回的是 hfdcan->ErrorCode —— 而 HAL_FDCAN_IRQHandler 是拿 **IR 寄存器**
     * 的位去 OR 它的 (Errors = Instance->IR & FDCAN_ERROR_MASK; ErrorCode |= Errors)。
     * BO/EP/EW/LEC 这些状态位在 **PSR** 寄存器里, 两者位号完全不同:
     *     FDCAN_IR_BO  = 0x00080000 (bit19)
     *     FDCAN_PSR_BO = 0x00000080 (bit7)
     * 而 FDCAN_ERROR_MASK 覆盖的位全在 bit17 以上, 永远命中不了 bit7。
     *
     * 后果: 一旦总线出问题把 TEC 顶到 256 进入 bus-off, 节点被踢下线,
     * Tx FIFO 里积压的帧再也发不出去 → can_error_step 恒为 1, 且**无法自恢复**。
     * ------------------------------------------------------------------ */
    if ((psr & FDCAN_PSR_BO) != 0u)
    {
        can_busoff_cnt++;

        /* bus-off 恢复需要 DeInit → MX_FDCAN2_Init → fdcan2_UserInit 整轮重初始化。
         * **刻意不在中断里做**: HAL_FDCAN_DeInit() 会关掉外设时钟并重配 RCC/GPIO/NVIC,
         * 在本中断自己的服务例程里重入 HAL 初始化风险很大 (见 CLAUDE.md V1.6.2 备注 2)。
         * 需要恢复的话, 由任务轮询 can_busoff_cnt 的变化后调用 fdcan2_recover()。 */
    }
}

/**
 * @brief  bus-off 后重新初始化 FDCAN2。**必须在任务上下文调用, 不能在中断里调用。**
 * @retval 1 成功, 0 失败
 */
uint8_t fdcan2_recover(void)
{
    HAL_FDCAN_DeInit(&hfdcan2);
    MX_FDCAN2_Init();
    fdcan2_UserInit();
    return 1;
}






