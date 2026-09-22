/**
 * @file    servo_scs.c
 * @brief   飞特(Feetech) SCS 总线舵机 —— 工程胶水层实现
 *
 * 本文件只做三件事：
 *   1. 实现厂商库要求使用方提供的三个硬件接口函数
 *      （ftUart_Send / ftUart_Read / ftBus_Delay，声明见 scslib/SCSerail.c）；
 *   2. 总线初始化：把 UART5 的波特率设成舵机用的值；
 *   3. 给厂商 API 套一层互斥保护，避免多任务同时占用总线。
 *
 * 协议打包/校验/应答全部在 scslib/ 里，本文件一行协议逻辑都没有。
 */

#include "Common_used.h"    /* libc + HAL + CMSIS-RTOS2 + huart5 (usart.h) */
#include "servo_scs.h"
#include "SCServo.h"        /* 厂商三套系列 API */

/* ============================================================
 * 内部状态
 * ============================================================ */

static osMutexId_t s_mutex = NULL;
static uint8_t     s_ready = 0;

static const osMutexAttr_t s_mutex_attr = {
    .name      = "scsBus",
    .attr_bits = osMutexPrioInherit,   /* 防止低优先级任务持锁拖住高优先级任务 */
    .cb_mem    = NULL,
    .cb_size   = 0U,
};

/* 诊断量，定义处在此，声明在 servo_scs.h */
volatile uint32_t g_scs_uart_tx_err   = 0;
volatile uint32_t g_scs_uart_rx_err   = 0;
volatile uint32_t g_scs_mutex_timeout = 0;

/* ============================================================
 * 厂商接缝：SCSLib 的硬件接口层
 *
 * scslib/SCSerail.c 里对这三个函数只有声明没有定义 —— 厂商 README 第 5 条
 * 要求「根据自己定义的串口修改 ftUart_Send/ftUart_Read」。本文件就是那个
 * 落点。协议层的所有收发最终都会走到这里。
 * ============================================================ */

/**
 * @brief  总线发送（厂商接口）。对应 scslib 的 wFlushSCS()。
 * @note   阻塞式，超时 SCS_UART_TX_TIMEOUT_MS。失败只记诊断量、不返回值
 *         —— 厂商把这个接口定成了 void，发送失败会在随后的应答读取里体现。
 */
void ftUart_Send(uint8_t *nDat, int nLen)
{
    if (nDat == NULL || nLen <= 0) {
        return;
    }
    if (HAL_UART_Transmit(&SCS_UART, nDat, (uint16_t)nLen, SCS_UART_TX_TIMEOUT_MS) != HAL_OK) {
        g_scs_uart_tx_err++;
    }
}

/**
 * @brief  总线接收（厂商接口）。对应 scslib 的 readSCS()。
 * @param  nDat  接收缓冲
 * @param  nLen  期望字节数
 * @retval nLen=成功收到全部字节；0=失败或超时
 * @note   阻塞式。厂商协议层把它当作「读到就返回、读不到返回 0」使用
 *         （见 scslib/SCS.c 的 checkHead()），所以这里必须返回 0 而不是 -1。
 */
int ftUart_Read(uint8_t *nDat, int nLen)
{
    if (nDat == NULL || nLen <= 0) {
        return 0;
    }
    if (HAL_UART_Receive(&SCS_UART, nDat, (uint16_t)nLen, SCS_UART_RX_TIMEOUT_MS) != HAL_OK) {
        g_scs_uart_rx_err++;
        return 0;
    }
    return nLen;
}

/**
 * @brief  总线静默延时（厂商接口）。对应 scslib 的 rFlushSCS()，每条命令前调用一次。
 * @note   厂商 demo 里是 HAL_Delay(1)。这里按上下文分流：
 *         **调度器未启动时必须走 HAL_Delay** —— osDelay() 在调度器启动前会
 *         NULL 解引用跳 HardFault（见 changelog V1.6.2 的定位结论）。
 */
void ftBus_Delay(void)
{
    if (osKernelGetState() == osKernelRunning) {
        osDelay(SCS_BUS_DELAY_MS);
    } else {
        HAL_Delay(SCS_BUS_DELAY_MS);
    }
}

/* ============================================================
 * 总线生命周期
 * ============================================================ */

uint8_t SCS_BusInit(void)
{
    s_ready = 0;

    /* huart5 是零初始化的全局量，CubeMX 没跑过 MX_UART5_Init() 时 Instance 为
       NULL。此时 GPIO/时钟/NVIC 全没配，本驱动无法自救 —— 如实返回失败，
       不要去碰 HAL。 */
    if (SCS_UART.Instance == NULL) {
        return 0;
    }

    /* 波特率在运行期改写，而不是去改 CubeMX 生成的 Core/Src/usart.c:91。
       理由有两条：
         1. 那一行在生成区内，CubeMX 重新生成会冲掉；
         2. 本工程 .ioc 里**没有 UART5.IPParameters 属性行**（USART1/2/3 都有），
            UART5 实为手工塞进 .ioc 的、CubeMX 并不真正管理，重新生成的行为
            不可预期 —— 不能把波特率托付给它。
       HAL_UART_Init() 在 gState != RESET 时**不会重跑 MspInit**，因此这里只
       重算 BRR，不会重配 GPIO / 时钟 / NVIC，是安全的原地改速率。
       PCLK1 = 170MHz，过采样 16 → BRR = 170，整除、零误差。 */
    if (SCS_UART.Init.BaudRate != SCS_UART_BAUD) {
        SCS_UART.Init.BaudRate = SCS_UART_BAUD;
        if (HAL_UART_Init(&SCS_UART) != HAL_OK) {
            return 0;
        }
    }

    /* 互斥锁的创建不依赖调度器是否启动（FreeRTOS 堆是静态数组，xQueueCreateMutex
       不碰 pxCurrentTCB），故无条件建一次。真正危险的是**取锁**，见 SCS_Lock()。 */
    if (s_mutex == NULL) {
        s_mutex = osMutexNew(&s_mutex_attr);
    }

    s_ready = 1;
    return 1;
}

uint8_t SCS_IsReady(void)
{
    return s_ready;
}

/* ============================================================
 * 互斥保护
 * ============================================================ */

int SCS_Lock(void)
{
    if (s_mutex == NULL) {
        return 0;
    }
    /* 调度器未启动时不能阻塞取锁：osMutexAcquire 带超时会走 xQueueSemaphoreTake
       的阻塞路径，触及尚未初始化的任务链表（同 V1.6.2 的 osDelay 问题）。 */
    if (osKernelGetState() != osKernelRunning) {
        return 0;
    }
    if (osMutexAcquire(s_mutex, SCS_MUTEX_TIMEOUT_MS) != osOK) {
        g_scs_mutex_timeout++;
        return 0;   /* 降级为无锁执行，而不是让舵机命令整条失败 */
    }
    return 1;
}

void SCS_Unlock(void)
{
    if (s_mutex != NULL) {
        (void)osMutexRelease(s_mutex);
    }
}

/* ============================================================
 * 命令包装
 *
 * 每个包装都是「取锁 → 调厂商函数 → 放锁」。取锁失败（未初始化 / 非任务
 * 上下文 / 超时）时 SCS_Lock() 返回 0，此时**只调用不放锁**，命令照发。
 * ============================================================ */

/* --- 通用 --- */

int SCS_Ping(uint8_t id)
{
    int held = SCS_Lock();
    int r = Ping(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_Reset(uint8_t id)
{
    int held = SCS_Lock();
    int r = Reset(id);
    if (held) { SCS_Unlock(); }
    return r;
}

/* --- SCSCL 系列 --- */

int SCS_WritePos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
    int held = SCS_Lock();
    int r = WritePos(id, position, time, speed);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_RegWritePos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
    int held = SCS_Lock();
    int r = RegWritePos(id, position, time, speed);
    if (held) { SCS_Unlock(); }
    return r;
}

void SCS_RegWriteAction(void)
{
    int held = SCS_Lock();
    RegWriteAction();
    if (held) { SCS_Unlock(); }
}

void SCS_SyncWritePos(uint8_t id[], uint8_t idn, uint16_t position[], uint16_t time[], uint16_t speed[])
{
    int held = SCS_Lock();
    SyncWritePos(id, idn, position, time, speed);
    if (held) { SCS_Unlock(); }
}

int SCS_EnableTorque(uint8_t id, uint8_t enable)
{
    int held = SCS_Lock();
    int r = EnableTorque(id, enable);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_ReadPos(uint8_t id)
{
    int held = SCS_Lock();
    int r = ReadPos(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_ReadMove(uint8_t id)
{
    int held = SCS_Lock();
    int r = ReadMove(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_FeedBack(int id)
{
    int held = SCS_Lock();
    int r = FeedBack(id);
    if (held) { SCS_Unlock(); }
    return r;
}

/* --- SMS_STS 系列 --- */

int SCS_WritePosEx(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
{
    int held = SCS_Lock();
    int r = WritePosEx(id, position, speed, acc);
    if (held) { SCS_Unlock(); }
    return r;
}

void SCS_SyncWritePosEx(uint8_t id[], uint8_t idn, int16_t position[], uint16_t speed[], uint8_t acc[])
{
    int held = SCS_Lock();
    SyncWritePosEx(id, idn, position, speed, acc);
    if (held) { SCS_Unlock(); }
}

int SCS_WheelMode(uint8_t id)
{
    int held = SCS_Lock();
    int r = WheelMode(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_SetMode(uint8_t id, uint8_t mode)
{
    int held = SCS_Lock();
    int r = SetMode(id, mode);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_WriteSpe(uint8_t id, int16_t speed, uint8_t acc)
{
    int held = SCS_Lock();
    int r = WriteSpe(id, speed, acc);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_CalibrationOfs(uint8_t id)
{
    int held = SCS_Lock();
    int r = CalibrationOfs(id);
    if (held) { SCS_Unlock(); }
    return r;
}

/* --- HLS 系列 --- */

int SCS_WritePosEx2(uint8_t id, int16_t position, uint16_t speed, uint8_t acc, uint16_t torque)
{
    int held = SCS_Lock();
    int r = WritePosEx2(id, position, speed, acc, torque);
    if (held) { SCS_Unlock(); }
    return r;
}

void SCS_SyncWritePosEx2(uint8_t id[], uint8_t idn, int16_t position[], uint16_t speed[], uint8_t acc[], uint16_t torque[])
{
    int held = SCS_Lock();
    SyncWritePosEx2(id, idn, position, speed, acc, torque);
    if (held) { SCS_Unlock(); }
}

int SCS_WriteSpeEx(uint8_t id, int16_t speed, uint8_t acc, uint16_t torque)
{
    int held = SCS_Lock();
    int r = WriteSpeEx(id, speed, acc, torque);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_EleMode(uint8_t id)
{
    int held = SCS_Lock();
    int r = EleMode(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_WriteEle(uint8_t id, int16_t torque)
{
    int held = SCS_Lock();
    int r = WriteEle(id, torque);
    if (held) { SCS_Unlock(); }
    return r;
}
