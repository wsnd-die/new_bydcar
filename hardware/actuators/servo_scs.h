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
 *
 * @warning **调用任务至少要有 1 KB 栈余量。** 厂商实现在栈上开了大数组：
 *          `SyncWritePosEx` / `SyncWriteSpe` 各有 `uint8_t offbuf[32*7]` = **224 字节**
 *          （scslib/SMS_STS.c:47、:116），再叠加回读函数的局部缓冲与
 *          `HAL_UART_Transmit/Receive` 的调用深度。本工程 `gripper` 任务当前只有
 *          **512 字节**栈（Core/Src/app_freertos.c:81），单独发几条 `SCS_WritePosEx`
 *          尚可，一旦用同步写、或加 `printf` 调试，会直接栈溢出。
 *          建议在 CubeMX 里把 `gripper` 栈提到 2048（改 `.ioc` —— 直接改
 *          `app_freertos.c` 会被重新生成冲掉）。
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

/**
 * @brief  **广播探测**：用广播 ID(0xFE) 发 Ping，任何 ID 的舵机都会应答。
 * @retval >0=应答舵机的真实 ID；0=总线无应答
 * @note   这是排查「总线上到底有没有东西」最有效的一招 —— 不必知道舵机 ID，
 *         也不必逐个 ID 试。协议层 `Ping()` 对 `ID==0xfe` 会跳过从机 ID 校验
 *         （见 scslib/SCS.c），因此任何在线舵机都会回包。
 */
uint8_t SCS_Probe(void);

/**
 * @brief  把 UART5 的底层状态拍一份快照，供诊断打印。
 * @param  isr_out  非空则写入 `UART5->ISR` 原始值
 * @param  cr1_out  非空则写入 `UART5->CR1` 原始值
 * @note   ISR 里值得看的位（STM32G4 参考手册）：
 *           bit3 ORE=1 溢出  bit1 FE=1 帧错误  bit2 NE=1 噪声  bit0 PE=1 校验错
 *           bit5 RXNE=1 收到字节  bit6 TC=1 发送完成  bit7 TXFNF=1 发送 FIFO 未满
 *         **若 ORE/FE/NE 有置位，说明 RX 线上确实有电平活动，只是采样时刻不对
 *         —— 那基本就是波特率不匹配。若这些位全 0 且一直收不到字节，
 *         说明 RX 线上压根没有信号（接线 / 供电 / 舵机没回）。**
 */
void SCS_BusSnapshot(uint32_t *isr_out, uint32_t *cr1_out);

/** @brief 当前设定的波特率 / UART5->BRR 实际值（核对时钟是否如预期）。 */
uint32_t SCS_BusBaud(void);
uint32_t SCS_BusBrr(void);

/**
 * @brief  **引脚通断测试**：不看 UART，直接把 PC12 当普通输出、PD2 当输入，
 *         量两根引脚是否电气相连。测完自动还原成 AF5。
 * @retval 2=拉低拉高都能读到(导线通)  1=只读到一个方向(悬空/虚接/被拽住)  0=完全断开
 * @note   与 `SCS_LoopbackTest` 的区别：这个**不依赖 UART 配置**，
 *         因此能把「线没接上」和「UART 收不了」彻底分开。
 */
uint8_t SCS_PinTieTest(void);

/** 自发自收的超时自旋次数（170MHz 下约 200000 ≈ 1.2ms，1M 波特率绰绰有余）。 */
#define SCS_LOOPBACK_TIMEOUT_LOOPS  200000u

/**
 * @brief  **自发自收自检**：先把 PC12 与 PD2 用杜邦线直接短接、断开舵机，再调用。
 * @param  n  发送字节数，建议 4~8
 * @retval 正确回读的字节数；==n 表示 UART5 的 TX/RX 通路完好
 * @note   这是把「MCU 侧」和「舵机/接线侧」切开的关键一招：
 *           ==n  → MCU 收发没问题, 问题在舵机那边(接线/信号/供电)
 *           ==0  → MCU 自己都听不到自己, 与舵机无关, 是引脚或外设配置问题
 */
uint8_t SCS_LoopbackTest(uint8_t n);

/**
 * @brief  **外部驱动判别**：PD2 这根脚是被外部电路驱动着，还是悬空的？
 * @retval 位标志：
 *           bit0 = 1  开内部下拉后仍读到高 → 外部在把该节点推向高
 *           bit1 = 1  开内部上拉后仍读到低 → 外部在把该节点拉向低
 *           == 0      两次都被内部电阻带跑 → 外部高阻，即**该脚悬空**
 * @note   补的是 `SCS_PinTieTest()` 的盲区：那个函数返回 1（"单向/悬空"）时，
 *         「PD2 压根没接线」和「PD2 接了、但被驱动板顶在某个电平上」会给出
 *         完全相同的结果，而这两者的排查方向正好相反 —— 前者去查线，
 *         后者去查驱动板/共地。本函数用内部上下拉（~40kΩ）把两者分开：
 *         40k 能把悬空节点随意拉向任一侧，却动不了有源驱动着的节点。
 * @warning 只能识别驱动能力明显强于 40k 的外部电路。外部若只是一只 47k 以上的
 *          弱上拉，会被内部下拉带跑而误报为悬空 —— 那种接法本来也带不动 UART，
 *          报悬空并不冤。
 * @warning 须在 `SCS_BusInit()` 之后调用（依赖 UART5 时钟已开）。
 *          函数结束时会把 PD2 还原成 UART5_RX(AF5)+上拉。
 */
uint8_t SCS_PinExternalDrive(void);

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
/**
 * @brief  取最近一次命令的错误码（见 SCS.h 的 SCS_ERR_LIST）。
 * @retval 0=无错  1=无应答  2=校验和不符  3=从机 ID 不符  4=长度不符
 */
int SCS_GetLastError(void);

/* --- 字节序（重要，见下）--- */

/**
 * @brief  声明后续**总线回读**命令面向的是哪一套字节序的舵机。
 * @param  big_endian 1 = 高字节在前（SCS/CL 系列，如 SCS0009）
 *                    0 = 低字节在前（SMS/STS、HLS 系列，如 STS3032）
 *
 * @note   **写命令不需要调** —— `SCS_WritePos()`（SCSCL）会自己切大端，
 *         `SCS_WritePosEx()`（SMS_STS）/ `SCS_WriteEx2()`（HLS）会自己切小端。
 *         需要显式调它的是**回读**：`SCS_ReadPos(id)` / `SCS_ReadSpeed(id)` 等
 *         在厂商库里是三套系列共用的实现，光凭 id 分不出该用哪套。
 *
 * @note   `SCS_FeedBack()` 只把原始字节搬进缓冲区，**不受本开关影响**；
 *         随后用 `-1` 从缓冲区取值的那条路也不受它影响（厂商写死了大端，
 *         即 SCS 系列正确、STS 系列取位置会错）。详见 scslib/SCSCL.c:111。
 */
void SCS_SetEnd(uint8_t big_endian);

/* 背景（飞特官方《通信协议高低字节问题》）：
 *     「协议中：SCS 系列高字节在前，SMS/STS 低字节在前」
 * 厂商库用一个全局 `End` 表示这件事，由 scslib/SCS.c 的 Host2SCS() 消费。
 * 本工程 UART5 上 ID1(STS3032) 与 ID2~6(SCS0009) 分属两个相反的系列，
 * 故每个系列专属的包装函数都在互斥锁内按需重设 —— 见 servo_scs.c 的说明。 */

/* --- EPROM 解锁（SMS_STS 系列专用，改 ID / 模式 / 限位前必须调用）--- */
/**
 * @brief  解锁 EPROM（写 SMS_STS_LOCK=55 为 0）。
 * @note   **只对 SMS_STS 系列（含 STS3032）有效** —— 内部写的是地址 55，
 *         而 SCSCL 系列的锁寄存器在地址 48，两者不可混用。
 *         典型用法见 SCS_LockEprom()。
 */
int SCS_UnlockEprom(uint8_t id);
/**
 * @brief  重新锁定 EPROM（写 SMS_STS_LOCK=55 为 1）。
 * @note   改 ID / 模式 / 角度限位等 **EPROM 区**寄存器必须按
 *         `SCS_UnlockEprom()` → 写 → `SCS_LockEprom()` 的次序成对使用，
 *         否则写入不生效（且不会有任何报错）。
 * @warning 厂商例程 `examples/SMS_STS/ProgramEprom.c` 在这里是**错的**：
 *          它调的是 SCSCL 的 `unLockEprom()`（写地址 48），而且解锁 ID 1
 *          却锁定 ID 2。不要照抄，用本函数的 Ex 版本。
 */
int SCS_LockEprom(uint8_t id);

/* --- SCSCL 系列（SCS/CL 舵机）--- */
/**
 * @brief 普通写位置。time/speed 为 0 表示用内部值。
 * @param position 目标位置。SCS0009 为 10 位编码器，**0~1024 对应 0~300°**
 *                 （产品页标注，分辨率 0.293°/步），故中位取 **500**。
 *                 部分教程按 0~1000 举例，两者都在量程内，不影响使用。
 * @note  本函数会先把总线字节序切成**大端**（SCS 系列），再发帧。
 */
int SCS_WritePos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed);
/** @brief 异步写位置（先缓存，收到 RegWriteAction 才执行）。 */
int SCS_RegWritePos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed);
/** @brief 触发所有已缓存的异步写。返回值无意义。 */
void SCS_RegWriteAction(void);
/** @brief 同步写多个舵机的位置（一条总线帧控制全部，最省带宽）。 */
void SCS_SyncWritePos(uint8_t id[], uint8_t idn, uint16_t position[], uint16_t time[], uint16_t speed[]);
/** @brief 扭矩使能，1=松/使能由舵机固件决定，0=卸力。 */
int SCS_EnableTorque(uint8_t id, uint8_t enable);
/** @brief 读当前位置。id>=0 走总线，id=-1 取 FeedBack 缓冲区；失败返回 -1。 */
int SCS_ReadPos(int id);
/** @brief 读移动状态。id>=0 走总线，id=-1 取 FeedBack 缓冲区；失败返回 -1。 */
int SCS_ReadMove(int id);
/**
 * @brief  一次性回读全部反馈量到厂商库的静态缓冲区。
 * @param  id  目标舵机 ID。
 * @retval >0=读到的字节数；-1=失败（看 SCS_GetLastError()）。
 * @note   之后用 `SCS_ReadPos(-1)` / `ReadSpeed(-1)` / `ReadLoad(-1)` /
 *         `ReadVoltage(-1)` / `ReadTemper(-1)` / `ReadMove(-1)` / `ReadCurrent(-1)`
 *         把这批量从缓冲区取出来，**不再产生总线往返**。
 *         —— 一次往返取 7 个量，比逐个 `SCS_ReadXxx(id)` 快 7 倍。
 * @warning **`id` 是 `int` 不是 `uint8_t` —— 传 -1 才有「取缓冲区」语义**，
 *          写成无符号类型会让 -1 变成 255，等于去 Ping 一个不存在的 ID 255。
 */
int SCS_FeedBack(int id);

/* --- SMS_STS 系列（SMS/STS 舵机，STS3032 属此系列）--- */
/**
 * @brief 写位置 + 速度 + 加速度（最常用：就是普通的位置控制）。
 * @param position 目标位置。12 位磁编码器 → **0~4095 对应 0~360°，中位 2048**。
 * @param speed    运行速度，**原始寄存器值**，单位随型号不同，见 STS3032 数据手册。
 *                 传 0 表示用寄存器里的既有值。
 * @param acc      加速度，**原始寄存器值**，0~254，0 表示不控加速度直冲最高速。
 * @note  内部一次写 7 字节，起始地址 SMS_STS_ACC(41)，覆盖 ACC / 目标位置 /
 *        运行时间 / 速度，即地址 41~47 全部由本函数一次写入。
 */
int SCS_WritePosEx(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);
/** @brief 异步写位置（先缓存，收到 SCS_RegWriteAction 才执行）。 */
int SCS_RegWritePosEx(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);
/**
 * @brief 同步写多个 SMS_STS 舵机的位置 —— 一条总线帧控制全部，最省带宽。
 * @warning **本函数在栈上开 `uint8_t offbuf[32*7]` = 224 字节**（厂商实现如此，
 *          见 scslib/SMS_STS.c:47）。调用它的任务栈**至少要有 1KB 余量**，
 *          否则直接栈溢出。见 servo_scs.h 的栈要求说明。
 */
void SCS_SyncWritePosEx(uint8_t id[], uint8_t idn, int16_t position[], uint16_t speed[], uint8_t acc[]);
/** @brief 切换到恒速（轮）模式。 */
int SCS_WheelMode(uint8_t id);
/** @brief 设置工作模式（见 SMS_STS.h 的模式常量）。 */
int SCS_SetMode(uint8_t id, uint8_t mode);
/** @brief 同步写多个 SMS_STS 舵机的恒速值。 */
void SCS_SyncWriteSpe(uint8_t id[], uint8_t idn, int16_t speed[], uint8_t acc[]);
/** @brief 恒速模式下写速度。 */
int SCS_WriteSpe(uint8_t id, int16_t speed, uint8_t acc);
/** @brief 中位校准。 */
int SCS_CalibrationOfs(uint8_t id);

/**
 * @brief  回读一组状态量。
 * @param  id  >=0 → 直接向该 ID 发读指令（一次总线往返，约 100µs）；
 *             **-1 → 从 SCS_FeedBack() 刚填好的缓冲区取**（不再上总线，零延时）。
 * @retval 读到的值；超时/校验失败返回 -1。
 * @note   两种用法：
 *             `SCS_FeedBack(1); pos = SCS_ReadPos(-1);`  ← 一次往返取全部量，推荐
 *             `pos = SCS_ReadPos(1);`                    ← 每次一个往返
 *         失败原因看 SCS_GetLastError()。
 */
int SCS_ReadSpeed(int id);
int SCS_ReadLoad(int id);
int SCS_ReadVoltage(int id);
int SCS_ReadTemper(int id);
int SCS_ReadCurrent(int id);

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
