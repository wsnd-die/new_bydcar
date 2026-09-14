/**
 * @file    HWT101_iic.c
 * @brief   HWT101 (WitMotion) 传感器完整驱动实现。
 *
 *          层次结构：
 *          ┌─────────────────────────────────┐
 *          │  HWT101_PollAngles / GetZeroYaw │  ← 应用层 API
 *          │  HWT101_HAL_Init               │
 *          ├─────────────────────────────────┤
 *          │  HWT101_I2cRead / I2cWrite     │  ← 协议适配层（回调）
 *          │  HWT101_RegUpdate              │
 *          ├─────────────────────────────────┤
 *          │  wit_protocol (WitReadReg …)   │  ← 协议栈
 *          ├─────────────────────────────────┤
 *          │  HWT101_ReadRegs (DMA+信号量)  │  ← 硬件抽象层
 *          │  HWT101_WriteReg (阻塞)       │
 *          │  HAL_I2C_Mem_Read_DMA          │
 *          └─────────────────────────────────┘
 *
 *          I2C1: PA15=SCL, PB9=SDA
 *          RX DMA: DMA1_Channel1 (hdma_i2c1_rx)
 *          TX:     阻塞 HAL_I2C_Mem_Write（配置帧极短，DMA 无优势）
 */

#include "Common_used.h"
#include "HWT101_iic.h"
#include "wit_protocol.h"

#ifdef HWT101_USE_I2C
#endif

#ifdef HWT101_USE_SERIAL
#endif

/* ========================================================================
   全局角度数据（解析后）
   ======================================================================== */

volatile float    g_hwt101_roll       = 0.0f;
volatile float    g_hwt101_pitch      = 0.0f;
volatile float    g_hwt101_yaw        = 0.0f;
volatile uint8_t  g_hwt101_data_ready = 0U;

/* ========================================================================
   内部状态
   ======================================================================== */

static float   s_fYawZero  = 0.0f;       /* 偏航角零偏                    */
static uint8_t s_bZeroSet  = 0U;         /* 零偏是否已记录                 */
  
#ifdef HWT101_USE_I2C
static SemaphoreHandle_t s_hI2cSem    = NULL;   /* DMA 完成信号量               */
static volatile HAL_StatusTypeDef s_I2cDmaResult; /* DMA 传输结果               */
#endif

/* ========================================================================
   内部常量
   ======================================================================== */

#define HWT101_CONVERT_SCALE  (180.0f / 32768.0f)   /* ±32768 → ±180°         */

/* ========================================================================
   1. 传输回调实现
   ======================================================================== */

#ifdef HWT101_USE_I2C

/* ---- I2C 读回调（DMA + 信号量同步） ---- */
static int32_t HWT101_I2cRead(uint8_t ucAddr, uint8_t ucReg,
                               uint8_t *p_ucData, uint16_t usLen)
{
    (void)ucAddr;  /* 地址已由 HWT101_ReadRegs 使用 HWT101_I2C_ADDR */
    if (HWT101_ReadRegs(ucReg, p_ucData, usLen) == HAL_OK)
        return 1;
    return 0;
}

/* ---- I2C 写回调（阻塞 — 短帧） ---- */
static int32_t HWT101_I2cWrite(uint8_t ucAddr, uint8_t ucReg,
                                uint8_t *p_ucData, uint16_t usLen)
{
    if (HAL_I2C_Mem_Write(&hi2c1, ucAddr, ucReg,
                           I2C_MEMADD_SIZE_8BIT,
                           p_ucData, usLen,
                           HWT101_I2C_TIMEOUT_MS) == HAL_OK)
        return 1;
    return 0;
}

#endif /* HWT101_USE_I2C */

#ifdef HWT101_USE_SERIAL

/* ---- 串口发送回调 ---- */
static void HWT101_SerialWrite(uint8_t *p_ucData, uint32_t uiLen)
{
    extern UART_HandleTypeDef huart4;
    HAL_UART_Transmit(&huart4, p_ucData, (uint16_t)uiLen, 100U);
}

/* ---- 串口帧校验和 ---- */
static uint8_t HWT101_CalcChecksum(uint8_t *data, uint16_t length)
{
    uint8_t sum = 0U;
    for (uint16_t i = 0U; i < length; i++) sum += data[i];
    return sum;
}

#endif /* HWT101_USE_SERIAL */

/* ---- 延时回调（两种模式共用） ---- */
static void HWT101_DelayMs(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

/* ---- 寄存器更新回调（解析角度） ---- */
static void HWT101_RegUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
    (void)uiRegNum;

    /* 角度寄存器 (Roll / Pitch / Yaw) 连续 3 字更新 */
    if (uiReg == Roll && uiRegNum >= 3U)
    {
        g_hwt101_roll  = (float)sReg[Roll]  * HWT101_CONVERT_SCALE;
        g_hwt101_pitch = (float)sReg[Pitch] * HWT101_CONVERT_SCALE;
        g_hwt101_yaw   = (float)sReg[Yaw]   * HWT101_CONVERT_SCALE;

        /* 首次有效值时记录零偏 */
        if (!s_bZeroSet)
        {
            s_fYawZero  = g_hwt101_yaw;
            s_bZeroSet  = 1U;
        }

        g_hwt101_data_ready = 1U;
    }
}

/* ========================================================================
   2. 底层 I2C（DMA 读 + 阻塞写）
   ======================================================================== */

#ifdef HWT101_USE_I2C

/* ---- 创建 DMA 同步信号量 ---- */
void HWT101_I2C_Init(void)
{
    if (s_hI2cSem == NULL)
    {
        s_hI2cSem = xSemaphoreCreateBinary();
        configASSERT(s_hI2cSem != NULL);
    }
}

/* ---- 在线检测（阻塞） ---- */
HAL_StatusTypeDef HWT101_IsReady(void)
{
    return HAL_I2C_IsDeviceReady(&hi2c1, HWT101_I2C_ADDR, 2U,
                                  HWT101_I2C_TIMEOUT_MS);
}

/* ---- 寄存器读取（DMA + 信号量同步） ---- */
HAL_StatusTypeDef HWT101_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if (s_hI2cSem == NULL)  return HAL_ERROR;
    if (data == NULL)       return HAL_ERROR;
    if (len == 0U)          return HAL_ERROR;

    /* 启动 DMA 读（TX 阶段由 HAL 内部 I2C 中断处理） */
    status = HAL_I2C_Mem_Read_DMA(&hi2c1, HWT101_I2C_ADDR, (uint16_t)reg,
                                   I2C_MEMADD_SIZE_8BIT, data, len);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 等待 DMA 完成信号量（带超时） */
    if (xSemaphoreTake(s_hI2cSem, pdMS_TO_TICKS(HWT101_I2C_TIMEOUT_MS)) != pdTRUE)
    {
        /*
         * 超时：传感器无响应或总线异常。
         * HAL 状态机已脏，调用者应重新初始化 I2C 外设。
         */
        return HAL_TIMEOUT;
    }

    return s_I2cDmaResult;
}

/* ---- 寄存器写入（阻塞） ---- */
HAL_StatusTypeDef HWT101_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, HWT101_I2C_ADDR, (uint16_t)reg,
                              I2C_MEMADD_SIZE_8BIT, &value, 1U,
                              HWT101_I2C_TIMEOUT_MS);
}

/* ---- DMA RX 完成回调（ISR 上下文 → 发信号量） ---- */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        s_I2cDmaResult = HAL_OK;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (s_hI2cSem != NULL)
        {
            xSemaphoreGiveFromISR(s_hI2cSem, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ---- I2C 错误回调（ISR 上下文 → 发信号量） ---- */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        s_I2cDmaResult = HAL_ERROR;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (s_hI2cSem != NULL)
        {
            xSemaphoreGiveFromISR(s_hI2cSem, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

#endif /* HWT101_USE_I2C */

/* ========================================================================
   3. 初始化
   ======================================================================== */

int32_t HWT101_HAL_Init(void)
{
    int32_t ret;

    /* Step 1 — 注册延时回调（共用） */
    ret = WitDelayMsRegister(HWT101_DelayMs);
    if (ret != WIT_HAL_OK) return ret;

    /* Step 2 — 注册数据更新回调 */
    ret = WitRegisterCallBack(HWT101_RegUpdate);
    if (ret != WIT_HAL_OK) return ret;

#ifdef HWT101_USE_I2C
    /* ---- I2C 模式 ---- */

    /* 2a. 注册 I2C 读写回调 */
    ret = WitI2cFuncRegister(HWT101_I2cWrite, HWT101_I2cRead);
    if (ret != WIT_HAL_OK) return ret;

    /* 2b. 初始化协议栈：I2C 协议，地址 0x50 */
    ret = WitInit(WIT_PROTOCOL_I2C, HWT101_SERIAL_ADDR);
    if (ret != WIT_HAL_OK) return ret;

    /* 2c. 输出内容：加速度 + 角速度 + 角度 */
    ret = WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE);
    if (ret != WIT_HAL_OK) return ret;

    /* 2d. 回传速率：50Hz */
    ret = WitSetOutputRate(RRATE_50HZ);
    if (ret != WIT_HAL_OK) return ret;

    /* 2e. 读取一次角度触发初始回调 */
    ret = WitReadReg(Roll, 3U);

    return ret;

#elif defined(HWT101_USE_SERIAL)
    /* ---- 串口模式 ---- */
    extern UART_HandleTypeDef huart4;

    /* 2a. 注册串口发送回调 */
    ret = WitSerialWriteRegister(HWT101_SerialWrite);
    if (ret != WIT_HAL_OK) return ret;

    /* 2b. 初始化协议栈：Normal 协议，地址 0x50 */
    ret = WitInit(WIT_PROTOCOL_NORMAL, HWT101_SERIAL_ADDR);
    if (ret != WIT_HAL_OK) return ret;

    /* 2c. 输出内容：加速度 + 角速度 + 角度 */
    ret = WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE);
    if (ret != WIT_HAL_OK) return ret;

    /* 2d. 回传速率：10Hz */
    ret = WitSetOutputRate(RRATE_10HZ);
    if (ret != WIT_HAL_OK) return ret;

    /* 2e. 使能 UART4 接收中断 */
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);

    return WIT_HAL_OK;
#endif
}

/* ========================================================================
   4. 运行期 API
   ======================================================================== */

/* ---- 在线检测 ---- */
int32_t HWT101_IsOnline(void)
{
#ifdef HWT101_USE_I2C
    return (HAL_I2C_IsDeviceReady(&hi2c1, HWT101_I2C_ADDR, 2U,
                                   HWT101_I2C_TIMEOUT_MS) == HAL_OK) ? 1 : 0;
#elif defined(HWT101_USE_SERIAL)
    /* 串口模式没有设备就绪检测，尝试读版本号确认 */
    if (WitReadReg(VERSION, 1U) == WIT_HAL_OK) return 1;
    return 0;
#endif
}

/* ---- 读取单个寄存器 ---- */
int16_t HWT101_ReadReg(uint32_t reg)
{
    if (reg >= REGSIZE) return 0;
    return sReg[reg];
}

/* ---- 相对偏航角 ---- */
float HWT101_GetZeroYaw(void)
{
    return g_hwt101_yaw - s_fYawZero;
}

/* ---- 主动轮询角度（I2C 模式） ---- */
int32_t HWT101_PollAngles(void)
{
#ifdef HWT101_USE_I2C
    return WitReadReg(Roll, 3U);
#else
    /* 串口模式由数据包自动推送，无需轮询 */
    return WIT_HAL_OK;
#endif
}

/* ========================================================================
   5. 串口帧解析
   ======================================================================== */

#ifdef HWT101_USE_SERIAL

void HWT101_ParsePacket(uint8_t *data)
{
    uint8_t cksum;
    int16_t raw;

    cksum = HWT101_CalcChecksum(data, 10U);
    if (cksum != data[10]) return;

    if (data[0] == 0x55U && data[1] == 0x53U)  /* 角度包 */
    {
        raw  = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
        g_hwt101_roll  = (float)raw * HWT101_CONVERT_SCALE;

        raw  = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
        g_hwt101_pitch = (float)raw * HWT101_CONVERT_SCALE;

        raw  = (int16_t)(((uint16_t)data[7] << 8) | data[6]);
        g_hwt101_yaw   = (float)raw * HWT101_CONVERT_SCALE;

        if (!s_bZeroSet)
        {
            s_fYawZero  = g_hwt101_yaw;
            s_bZeroSet  = 1U;
        }

        g_hwt101_data_ready = 1U;
    }
}

void HWT101_FeedSerialByte(uint8_t data)
{
    WitSerialDataIn(data);
}

#else

void HWT101_ParsePacket(uint8_t *data)
{
    (void)data;
}

void HWT101_FeedSerialByte(uint8_t data)
{
    (void)data;
}

#endif /* HWT101_USE_SERIAL */