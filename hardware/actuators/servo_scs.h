/**
 * @file    servo_scs.h
 * @brief   飞特(Feetech) SCS 总线舵机 —— 工程胶水层（UART5, PC12=TX / PD2=RX）
 *
 * 厂商库按原样收在 hardware/actuators/scslib/，**代码零改动**，只做了
 * GBK→UTF-8 的编码归一（源包里 5 个文件是 GBK、6 个是 UTF-8）。
 *
 * 厂商把「硬件接口」抽象成三个由使用方实现的函数，声明在 scslib/SCSerail.c：
 *      void ftUart_Send(uint8_t *nDat, int nLen);
 *      int  ftUart_Read (uint8_t *nDat, int nLen);
 *      void ftBus_Delay (void);
 * 本文件就是这三个函数的唯一落点，外加总线初始化与互斥保护。
 * 厂商 README 第 5 条要求「根据自己定义的串口修改 ftUart_Send/ftUart_Read」，
 * 指的就是这件事。
 *
 * @note    分层：本文件属**硬件驱动层**（CLAUDE.md 第 1 节）。scslib/ 内是纯协议
 *          实现，不含任何 HAL 调用；所有 HAL 相关代码集中在本文，换外设只改这里。
 *
 * @warning 直接调用 scslib/ 里的裸 API（`WritePos` / `Ping` / `Reset` …）会**绕过**
 *          本层的互斥保护。多任务环境下请优先用下面的 `SCS_*` 包装函数；确需调用
 *          裸 API 时，用 `SCS_LOCKED(...)` 宏或 `SCS_Lock()/SCS_Unlock()` 自行包围。
 *
 * @warning 本驱动用的是**阻塞式**收发（`HAL_UART_Transmit` / `HAL_UART_Receive`），
 *          与工程里其它串口驱动（走 IT/DMA）不同。这是有意的：厂商协议是
 *          「发一帧→收一帧」的严格时序，阻塞实现最简单也最容易验证。代价是
 *          **舵机不应答时每字节耗时 `SCS_UART_RX_TIMEOUT_MS`**，因此本驱动只应在
 *          低频命令任务里调用，不要放进控制环。
 */
#ifndef SERVO_SCS_H
#define SERVO_SCS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 可调参数
 * ============================================================ */

/** 舵机总线所用串口。UART5 = PC12(TX) / PD2(RX)，工程里空闲。 */
#define SCS_UART                huart5

/**
 * 总线波特率。飞特 SCS/STS 系列出厂默认 1M。
 * @note 本值在运行期由 SCS_BusInit() 写进 huart5.Init.BaudRate 并重调
 *       HAL_UART_Init()，**不依赖 CubeMX 生成的 usart.c 里那个 115200**
 *       （原因见 SCS_BusInit 注释）。若舵机被改过波特率，只改这里即可。
 *       详见 INST.h 的 SCS_1M / SCS_0_5M / SCS_250K … 常量表。
 */
#define SCS_UART_BAUD           1000000U

/** 发送超时（ms）。一帧最长 128 字节，1M 波特率下约 1.3ms。 */
#define SCS_UART_TX_TIMEOUT_MS  100U

/**
 * 接收超时（ms），**单字节**语义。
 * @warning 这是失败路径的代价：舵机不在线时，协议层 checkHead() 读第一个字节
 *          就会等满这个时长。总线上有 6 个舵机、其中任何一个掉线，一条命令就
 *          多花这么多毫秒。调小可以加快失败返回，但太小会误判应答为超时。
 */
#define SCS_UART_RX_TIMEOUT_MS  100U

/**
 * 每条命令前的总线静默延时（ms），供协议层 rFlushSCS() 调用。
 * 对应厂商 demo 里的 HAL_Delay(1)。
 */
#define SCS_BUS_DELAY_MS        1U

/** 取互斥锁的最长等待（ms）。超时会记进 g_scs_mutex_timeout 并降级为无锁执行。 */
#define SCS_MUTEX_TIMEOUT_MS    200U

/* ============================================================
 * 诊断量（参考 Core/Src/can.c 的 can_error_step 模式）
 * ============================================================ */

extern volatile uint32_t g_scs_uart_tx_err;      /**< HAL_UART_Transmit 失败次数 */
extern volatile uint32_t g_scs_uart_rx_err;      /**< HAL_UART_Receive 失败/超时次数 */
extern volatile uint32_t g_scs_mutex_timeout;    /**< 取锁超时次数（>0 说明总线被抢） */

/* ============================================================
 * 总线生命周期
 * ============================================================ */

/**
 * @brief  初始化舵机总线：挂接 UART5、设定波特率、建互斥锁。
 * @note   应在任务上下文中调用一次。**可以重复调用**（幂等）。
 *         本函数不配置 GPIO/NVIC —— 那由 CubeMX 生成的 MX_UART5_Init() 完成。
 * @retval 1=就绪  0=失败（uart 未初始化）
 */
uint8_t SCS_BusInit(void);

/**
 * @brief  总线是否已初始化。
 * @retval 1=就绪  0=未初始化（此时 SCS_* 命令会走无锁路径，仍可发但无保护）
 */
uint8_t SCS_IsReady(void);

/* ============================================================
 * 互斥保护
 * ============================================================ */

/**
 * @brief  取总线互斥锁。取不到（未初始化/非任务上下文/超时）会记诊断量并放行，
 *         即**降级为无锁执行**而不是返回失败 —— 宁可少一层保护，也不要让舵机
 *         命令因为锁的问题整条发不出去。
 * @retval 1=已持锁，调用方须配对 SCS_Unlock()  0=未持锁，无需解锁
 */
int SCS_Lock(void);

/** @brief 释放总线互斥锁。必须与返回 1 的 SCS_Lock() 配对。 */
void SCS_Unlock(void);

/**
 * @brief 用互斥锁包围一次任意厂商调用（含未包装的裸 API）。
 * @param call 任意厂商函数调用，如 SCS_LOCKED(RegWriteAction());
 * @note  只用于**不需要返回值**的调用；需要返回值请用下面的 SCS_* 包装函数。
 */
#define SCS_LOCKED(call)                        \
    do {                                        \
        int _scs_held = SCS_Lock();             \
        (void)(call);                           \
        if (_scs_held) { SCS_Unlock(); }        \
    } while (0)

/* ============================================================
 * 命令包装（已加锁）—— 对应 scslib 的三套系列
 * ============================================================ */

/* --- 通用：任意系列都可用 --- */
/** @brief Ping：舵机在线返回其 ID，超时返回 -1。 */
int SCS_Ping(uint8_t id);
/** @brief 复位舵机（回到中位）。成功返回 ID，超时返回 -1。 */
int SCS_Reset(uint8_t id);

/* --- SCSCL 系列（SCS/CL 舵机）--- */
/** @brief 普通写位置。position 0~1000 对应 0~300°，time/speed 为 0 表示用内部值。 */
int SCS_WritePos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed);
/** @brief 异步写位置（先缓存，收到 RegWriteAction 才执行）。 */
int SCS_RegWritePos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed);
/** @brief 触发所有已缓存的异步写。返回值无意义。 */
void SCS_RegWriteAction(void);
/** @brief 同步写多个舵机的位置（一条总线帧控制全部，最省带宽）。 */
void SCS_SyncWritePos(uint8_t id[], uint8_t idn, uint16_t position[], uint16_t time[], uint16_t speed[]);
/** @brief 扭矩使能，1=松/使能由舵机固件决定，0=卸力。 */
int SCS_EnableTorque(uint8_t id, uint8_t enable);
/** @brief 读当前位置，超时返回 -1。 */
int SCS_ReadPos(uint8_t id);
/** @brief 读移动状态，超时返回 -1。 */
int SCS_ReadMove(uint8_t id);
/** @brief 读全部反馈（位置/速度/负载/电压/温度/移动/电流）。 */
int SCS_FeedBack(int id);

/* --- SMS_STS 系列（SMS/STS 舵机）--- */
/** @brief 写位置 + 速度 + 加速度。position 为有符号值，中位 2048。 */
int SCS_WritePosEx(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);
/** @brief 同步写多个 SMS_STS 舵机的位置。 */
void SCS_SyncWritePosEx(uint8_t id[], uint8_t idn, int16_t position[], uint16_t speed[], uint8_t acc[]);
/** @brief 切换到恒速（轮）模式。 */
int SCS_WheelMode(uint8_t id);
/** @brief 设置工作模式（见 SMS_STS.h 的模式常量）。 */
int SCS_SetMode(uint8_t id, uint8_t mode);
/** @brief 恒速模式下写速度。 */
int SCS_WriteSpe(uint8_t id, int16_t speed, uint8_t acc);
/** @brief 中位校准。 */
int SCS_CalibrationOfs(uint8_t id);

/* --- HLS 系列（HLS 舵机）--- */
/** @brief 写位置 + 速度 + 加速度 + 扭矩。 */
int SCS_WritePosEx2(uint8_t id, int16_t position, uint16_t speed, uint8_t acc, uint16_t torque);
/** @brief 同步写多个 HLS 舵机的位置。 */
void SCS_SyncWritePosEx2(uint8_t id[], uint8_t idn, int16_t position[], uint16_t speed[], uint8_t acc[], uint16_t torque[]);
/** @brief 恒速模式写速度 + 扭矩。 */
int SCS_WriteSpeEx(uint8_t id, int16_t speed, uint8_t acc, uint16_t torque);
/** @brief 切换到恒流模式。 */
int SCS_EleMode(uint8_t id);
/** @brief 恒流模式下写电流。 */
int SCS_WriteEle(uint8_t id, int16_t torque);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_SCS_H */
