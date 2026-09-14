#include "Common_used.h"

TBData_t TB_position = {0};
TBData_t TB_speed = {0};
float imu_gz = 0;
float imu_yaw = 0;
uint8_t Flag_TBOFdata = 0;

/* ---- OpenMV 颜色帧接收 (UART2, AA l_black l_mean A B DD 6字节) ---- */
volatile uint8_t g_uart2_color_ready = 0;
uint8_t g_uart2_color_l, g_uart2_color_l_mean, g_uart2_color_a, g_uart2_color_b;

/* ---- GY-33 颜色帧接收 (UART2, 5A 5A type qty data[qty] chk) ---- */
volatile uint8_t g_uart2_gy33_ready = 0;
uint8_t g_uart2_gy33_r, g_uart2_gy33_g, g_uart2_gy33_b;

/* GY-33 帧解析状态机 (可跨 DMA 缓冲拆包) */
static uint8_t gy33_rx_buf[16];
static uint8_t gy33_rx_idx = 0;
static uint8_t gy33_rx_state = 0;   /* 0=等5A, 1=等第2个5A, 2=收数据 */

static void UART2_Gy33ParseByte(uint8_t b)
{
    uint8_t type, qty, sum;

    switch (gy33_rx_state) {
    case 0:
        if (b == 0x5A) { gy33_rx_buf[0] = b; gy33_rx_state = 1; }
        break;
    case 1:
        if (b == 0x5A) { gy33_rx_buf[1] = b; gy33_rx_idx = 2; gy33_rx_state = 2; }
        else gy33_rx_state = (b == 0x5A) ? 1 : 0;
        break;
    case 2:
        gy33_rx_buf[gy33_rx_idx++] = b;
        if (gy33_rx_idx < 4) break;              /* 还没收到 type+qty */
        type = gy33_rx_buf[2];
        qty  = gy33_rx_buf[3];
        if (qty <= 12 && gy33_rx_idx >= (4u + qty + 1u)) {   /* 帧完整 */
            sum = 0x5A + 0x5A + type + qty;
            for (uint8_t i = 0; i < qty; i++) sum += gy33_rx_buf[4 + i];
            if ((sum & 0xFF) == gy33_rx_buf[4 + qty]) {
                if (type == 0x45 && qty == 3) {   /* RGB */
                    g_uart2_gy33_r = gy33_rx_buf[4];
                    g_uart2_gy33_g = gy33_rx_buf[5];
                    g_uart2_gy33_b = gy33_rx_buf[6];
                    g_uart2_gy33_ready = 1;
                }
            }
            gy33_rx_state = 0; gy33_rx_idx = 0;
        } else if (gy33_rx_idx >= 16) {           /* 溢出保护 */
            gy33_rx_state = 0; gy33_rx_idx = 0;
        }
        break;
    }
}

static void UART2_ScanColorFrame(const uint8_t *data, uint16_t len)
{
    /* 扫描 AA l_black l_mean A B DD */
    for (uint16_t i = 0; i + 5 < len; i++) {
        if (data[i] == 0xAA && data[i + 5] == 0xDD) {
            if (data[i + 2] <= 100) {  /* l_mean 校验: Lab L 范围 0~100 */
                g_uart2_color_l      = data[i + 1];  /* 黑色像素占比×255 */
                g_uart2_color_l_mean = data[i + 2];  /* Lab L 0~100 */
                g_uart2_color_a      = data[i + 3];  /* Lab A+128 */
                g_uart2_color_b      = data[i + 4];  /* Lab B+128 */
                g_uart2_color_ready  = 1;
            }
        }
    }
}

#if LEGACY_USART2_ODOM_ENABLE
UART_STATE fsm_state = WAIT_HEADER1;
uint8_t frame_buf[FRAME_TOTAL_LEN];
uint8_t frame_index = 0;
uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];

void UART2_FSM_Parse_Byte(uint8_t byte)
{
    switch (fsm_state) {
    case WAIT_HEADER1:
        if (byte == TB_Header1) {
            frame_buf[0] = byte;
            fsm_state = WAIT_HEADER2;
        }
        break;
    case WAIT_HEADER2:
        if (byte == TB_Header2) {
            frame_buf[1] = byte;
            frame_index = 2;
            fsm_state = RECV_DATA;
        } else {
            fsm_state = WAIT_HEADER1;
        }
        break;
    case RECV_DATA:
        frame_buf[frame_index++] = byte;
        if (frame_index == FRAME_TOTAL_LEN - 2) {
            fsm_state = CHECK_TAIL1;
        }
        break;
    case CHECK_TAIL1:
        if (byte == TB_Tail1) {
            frame_buf[FRAME_TOTAL_LEN - 2] = byte;
            fsm_state = CHECK_TAIL2;
        } else {
            frame_index = 0;
            fsm_state = WAIT_HEADER1;
        }
        break;
    case CHECK_TAIL2:
        if (byte == TB_Tail2) {
            FloatUnion tmp;
            memcpy(tmp.data, &frame_buf[2], 4);
            TB_position.xdata = tmp.val;
            memcpy(tmp.data, &frame_buf[6], 4);
            TB_position.ydata = tmp.val;
            memcpy(tmp.data, &frame_buf[10], 4);
            TB_speed.xdata = tmp.val;
            memcpy(tmp.data, &frame_buf[14], 4);
            TB_speed.ydata = tmp.val;
            memcpy(tmp.data, &frame_buf[18], 4);
            imu_gz = tmp.val;
            memcpy(tmp.data, &frame_buf[22], 4);
            imu_yaw = tmp.val;
            Flag_TBOFdata = 1;
        }
        frame_index = 0;
        fsm_state = WAIT_HEADER1;
        break;
    }
}

volatile uint32_t dbg_rx_cb = 0;  /* CALLBACK 入口计数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    dbg_rx_cb++;
    if (huart->Instance == USART2) {
        /* 扫描 OpenMV 颜色帧 AA L A B DD */
        UART2_ScanColorFrame(dma_rx_buf, Size);
        /* 扫描里程计帧 AA CC ... BB DD */
        for (uint16_t i = 0; i < Size; i++) {
            UART2_FSM_Parse_Byte(dma_rx_buf[i]);
            UART2_Gy33ParseByte(dma_rx_buf[i]);   /* GY-33 颜色帧 5A 5A ... */
        }
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* USART1(扫码口): 错误后必须重挂接收, 否则 QR_Flag 永远不置位,
         * Qr_Get() 无限等待, 扫码后不执行下一步 */
        QRcode_UART_ErrorCallback(huart);
    }
    else if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);
    }
    else if (huart->Instance == USART3) {
        K230_RxRestart();
    }
}
#endif

void UART2_StartDMAReceive(void)
{
#if LEGACY_USART2_ODOM_ENABLE
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);
#endif
}

/* ���첢���͵��ֽ�ָ��֡ */
static void UART2_SendCmd(uint8_t cmd)
{
    uint8_t tx_buf[4];
    uint8_t idx = 0;
    tx_buf[idx++] = TB_Header1;
    tx_buf[idx++] = TB_Header2;
    tx_buf[idx++] = cmd;
    tx_buf[idx++] = TB_Tail1;
    tx_buf[idx++] = TB_Tail2;
    HAL_UART_Transmit(&huart2, tx_buf, idx, HAL_MAX_DELAY);
}

/* ����������������ֵ��̣�����������ƽ�ƣ����ȴ�������� */
static void Mecanum_MoveBlocking(float body_dx_m, float body_dy_m)
{
    MecanumMove_t move;
    if (Mecanum_CalculateMove(&g_mecanum_config, body_dx_m, body_dy_m, 0.0f, &move))
    {
        Mecanum_ExecuteMove(&g_mecanum_config, &move);
        osDelay((uint32_t)(move.duration_s * 1000.0f) + 50U);
    }
}

void UART2_calibrate(void)
{

    UART2_SendCmd('x');                       /* AA CC x BB DD */
    osDelay(1000U);                           /* �ȴ� 1 �� */

    Mecanum_MoveBlocking(1.0f, 0.0f);         /* ǰ�� 1m������ +X Ϊǰ�� */

    UART2_SendCmd('z');                       /* AA CC z BB DD */
    osDelay(1000U);                           /* �ȴ� 1 �� */

    UART2_SendCmd('y');                       /* AA CC y BB DD */
    osDelay(1000U);                           /* �ȴ� 1 �� */

    Mecanum_MoveBlocking(0.0f, -1.0f);        /* ˮƽ���� 1m������ +Y Ϊ�󣬹���Ϊ -Y�� */
    osDelay(1000U);

    UART2_SendCmd('z');                       /* AA CC z BB DD */
    osDelay(1000U);                           /* �ȴ� 1 �� */
}

