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
 *
 * @note   **实现与厂商 demo 一致：全程不碰 TE。**
 *         厂商 FTServo_stm32HAL-main/Core/Src/main.c:71 的 ftUart_Send()
 *         就只是一句 HAL_UART_Transmit()；TE 由 HAL_UART_Init(Mode=TX_RX)
 *         设好之后一直常开。这里照做。
 *
 *         本函数原先在每个帧前后 SET_BIT/CLEAR_BIT(USART_CR1_TE)，理由是
 *         「不清 TE 的话 PC12 空闲时被 push-pull 强驱动高电平，舵机回包拉不低
 *         这条线」。那个理由成立的前提是**单线半双工** —— PC12 与 PD2 并接到
 *         同一根信号线。实测 SCS_PinTieTest() 在本工程所用的总线舵机驱动板上
 *         稳定返回 1/2，即 PC12 的驱动传不到 PD2：板子把 TX/RX 分成了两路，
 *         该前提不成立。
 *
 *         反证更直接：能正常驱动这套舵机的厂商 FD 上位机走的是 USB-TTL，
 *         而 USB-TTL 的 TX 本就是永久 push-pull 空闲高，与「TE 常开」完全
 *         等价；硬件既然吃得消那种接法，清 TE 就只剩代价 —— STM32 每次 TE
 *         由 0 置 1 都会先发一个 idle 帧（一整帧全 1）再发数据，而原实现
 *         除第一帧外每帧都要翻转一次 TE。
 *
 *         @warning 不要再在收发路径上翻转 TE。发完就清会让**第二帧起根本发
 *         不出去**（TE 为 0 时数据写进 TDR 也移位不出去，HAL_UART_Transmit()
 *         会等满超时才返回），表现为 tx_err 暴涨、总线全哑。
 */
void ftUart_Send(uint8_t *nDat, int nLen)
{
    if (nDat == NULL || nLen <= 0) {
        return;
    }
    if (HAL_UART_Transmit(&SCS_UART, nDat, (uint16_t)nLen, SCS_UART_TX_TIMEOUT_MS) != HAL_OK) {
        g_scs_uart_tx_err   ++;
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

    /* 清掉上电以来积压的错误标志。
       HAL_UART_Init() **不清这些位** —— 核过 stm32g4xx_hal_uart.c:307-374，
       它只重配 CR1/CR2/CR3/PRESC/BRR，全程没有任何 __HAL_UART_CLEAR_FLAG。
       而 PD2 在 MX_UART5_Init() 把它设成 AF5 之前是一个悬空输入脚，上电噪声
       完全可能在那一刻就把 FE / IDLE 置上，并一路留到现在（之后的轮询接收只在
       RXNE 置位时才读 RDR，碰不到这些位）。
       不清的话，SCS_BusSnapshot() 读到的就可能是**上电残留**而不是自检期间的
       真实情况 —— 于是「FE=1」既可能是旧账，也可能是现场，无法定性。
       清一次，让后续诊断只反映本次运行。 */
    __HAL_UART_CLEAR_FLAG(&SCS_UART, UART_CLEAR_FEF | UART_CLEAR_NEF |
                                      UART_CLEAR_PEF | UART_CLEAR_OREF |
                                      UART_CLEAR_IDLEF);

    /* PD2 内部上拉（~40k）。
       原有注释写的是「半双工兜底：PC12 发送完变高阻后靠它维持空闲高」，那个
       场景随 ftUart_Send() 的改动已经不存在了：现在 TE 常开，PC12 一直是
       push-pull 输出，不存在「释放期间」。而实测本工程用的总线舵机驱动板把
       TX/RX 分成了两路（SCS_PinTieTest() 返回 1/2），PC12 的电平本也到不了
       PD2 —— 那块板子会自己驱动 PD2。

       保留这行是因为它无害且是防御性的：只有当板子的 RX 输出是高阻（开漏
       未上拉 / 板子未上电）时它才起作用，那时它给 PD2 一个确定的空闲高电平，
       比悬空强。厂商 demo 的 MspInit 是 NOPULL。若日后确认板子恒为推挽驱动，
       删掉也不影响。
       放在运行期做而不是改 usart.c 的 MspInit：那在 CubeMX 生成区，会被冲掉。 */
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin       = GPIO_PIN_2;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF5_UART5;
        HAL_GPIO_Init(GPIOD, &gpio);
    }

    /* 字节序的**初值**：小端（低字节在前），即 SMS_STS/HLS 系列。
       SCS.c 里 End 的静态初值本来就是 0，所以这行是幂等的空操作 —— 显式写出来
       是为了防止将来有人改动 SCS.c 的初值而没人注意到 STS 系列会因此字节序反转。

       @warning 这**不是**一次性的全局设定。本工程一条总线上混了两种字节序的系列
                （ID1 STS3032 小端 / ID2~6 SCS0009 大端），每个系列专属的命令包装
                都会在锁内重新调用 scs_big_endian() / scs_little_endian()。
                详见本文件「命令包装」上方的字节序说明。 */
    setEnd(0);

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

uint8_t SCS_Probe(void)
{
    int r = SCS_Ping(0xFE);      /* 广播 ID：任何在线舵机都会应他自己的 ID */
    return (r > 0 && r != 0xFE) ? (uint8_t)r : 0;
}

void SCS_BusSnapshot(uint32_t *isr_out, uint32_t *cr1_out)
{
    if (isr_out != NULL) { *isr_out = (uint32_t)SCS_UART.Instance->ISR; }
    if (cr1_out != NULL) { *cr1_out = (uint32_t)SCS_UART.Instance->CR1; }
}

uint32_t SCS_BusBaud(void)
{
    return (uint32_t)SCS_UART.Init.BaudRate;
}

uint32_t SCS_BusBrr(void)
{
    return (uint32_t)SCS_UART.Instance->BRR;
}

uint8_t SCS_PinTieTest(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t low_ok = 0, high_ok = 0;

    /* 把两根引脚从 AF 抢回来当普通 IO：PC12 推挽输出，PD2 输入 */
    gpio.Pin   = GPIO_PIN_2;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);

    gpio.Pin  = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* 拉低 → PD2 应该读到低 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    for (volatile uint32_t i = 0; i < 20000u; i++) { }
    low_ok = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_RESET) ? 1u : 0u;

    /* 拉高 → PD2 应该读到高 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    for (volatile uint32_t i = 0; i < 20000u; i++) { }
    high_ok = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET) ? 1u : 0u;

    /* 还原成 UART5 的 AF5，否则后面所有收发都废了 */
    gpio.Pin       = GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Alternate = GPIO_AF5_UART5;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin       = GPIO_PIN_2;
    gpio.Pull      = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &gpio);

    return (uint8_t)(low_ok + high_ok);   /* 2=通, 1=单向/悬空, 0=彻底断开 */
}

/**
 * @brief  **外部驱动判别**：PD2 这个节点是被外部电路驱动着，还是悬空的。
 * @retval 位标志（见头文件）：
 *           bit0 = 1  开内部下拉后仍读到高 → 外部在把该节点推向高
 *           bit1 = 1  开内部上拉后仍读到低 → 外部在把该节点拉向低
 *           == 0      两次都被内部电阻带跑 → 外部高阻，即**该脚悬空**
 * @note   存在的理由：`SCS_PinTieTest()` 返回 1（"单向/悬空"）时无法区分
 *         「PD2 压根没接线」和「PD2 接了，但被驱动板顶在某个电平上」——
 *         而这两者对应的排查方向完全相反。
 *
 *         原理：STM32 内部上下拉约 40kΩ，是非常弱的驱动。
 *           - 节点上什么都没接 → 40k 轻松把它拉到想要的方向；
 *           - 节点接在有源电路上 → 对方的推挽输出（或强上拉）无视这 40k，
 *             把它顶在原本的电平上。
 *         所以「开下拉却仍读到高」是**这根脚确实接在有源电路上**的证据；
 *         两次都被内部电阻带跑则是**这根脚悬空**的证据。
 *
 * @warning 只能识别「驱动能力明显强于 40k」的外部电路。若外部只是一只
 *          47k 以上的弱上拉，会被 40k 内部下拉带跑，误报为悬空 —— 那种接法
 *          本来也驱动不了 UART，报悬空并不冤。
 * @warning 必须在 UART5 时钟已开时调用（`SCS_BusInit()` 之后）。
 *          本函数结束时会把 PD2 还原成 UART5_RX(AF5) + 上拉。
 */
uint8_t SCS_PinExternalDrive(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t with_pullup   = 0;   /* 内部上拉时的读数 */
    uint8_t with_pulldown = 0;   /* 内部下拉时的读数 */

    gpio.Pin   = GPIO_PIN_2;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    /* 1) 内部上拉：悬空节点应被拉高 */
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &gpio);
    for (volatile uint32_t i = 0; i < 20000u; i++) { }
    with_pullup = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET) ? 1u : 0u;

    /* 2) 内部下拉：悬空节点应被拉低 */
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOD, &gpio);
    for (volatile uint32_t i = 0; i < 20000u; i++) { }
    with_pulldown = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET) ? 1u : 0u;

    /* 还原成 UART5_RX。与 SCS_BusInit() 里的配置保持一致，
       否则后面所有接收都废了。 */
    gpio.Pin       = GPIO_PIN_2;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Alternate = GPIO_AF5_UART5;
    HAL_GPIO_Init(GPIOD, &gpio);

    return (uint8_t)((with_pulldown ? 1u : 0u) | (with_pullup ? 0u : 2u));
}

/**
 * @brief  自发自收自检：**必须先把 PC12 与 PD2 用杜邦线直接短接**，并断开舵机。
 * @param  n  发送字节数（建议 4~8）
 * @retval 正确回读的字节数；==n 表示 UART 的 TX/RX 通路完好
 * @note   轮询实现而不是 HAL_UART_Transmit + HAL_UART_Receive —— 后者是
 *         「先发完再武装接收」，自己发的字节在自己武装之前就跑光了，永远收不到。
 *         这里逐字节「发一个、立刻读一个」，才能抓住自己的回声。
 */
uint8_t SCS_LoopbackTest(uint8_t n)
{
    static const uint8_t pat[] = {0xA5u, 0x5Au, 0x3Cu, 0xC3u,
                                  0x0Fu, 0xF0u, 0x96u, 0x69u};
    uint8_t ok = 0;

    if (n > sizeof(pat)) { n = (uint8_t)sizeof(pat); }

    /* TE 由 HAL_UART_Init(Mode=TX_RX) 设好之后常开，这里不再自己翻转。
       原先本函数结尾有一句 CLEAR_BIT(TE)，那会让**测完之后整个总线都发不出帧**
       —— 见 ftUart_Send() 的 @warning。 */

    /* 清掉可能残留的字节，否则读到的不是自己的回声 */
    while (__HAL_UART_GET_FLAG(&SCS_UART, UART_FLAG_RXNE)) {
        (void)SCS_UART.Instance->RDR;
    }

    for (uint8_t i = 0; i < n; i++) {
        uint32_t guard;

        guard = SCS_LOOPBACK_TIMEOUT_LOOPS;
        while (!__HAL_UART_GET_FLAG(&SCS_UART, UART_FLAG_TXE) && guard--) { }
        SCS_UART.Instance->TDR = pat[i];

        guard = SCS_LOOPBACK_TIMEOUT_LOOPS;
        while (!__HAL_UART_GET_FLAG(&SCS_UART, UART_FLAG_RXNE) && guard--) { }

        /* 判据必须是 RXNE 本身，**不能判 guard**。
           这里原先是 `if (guard == 0) { break; }`，那是一段死代码：守卫是
           后置自减，超时退出时 `guard--` 刚好把 0 减成 0xFFFFFFFF，所以
           `guard == 0` 恒为假。后果是早退永不发生（每字节都等满整段超时），
           而且超时后仍会掉进下面那句读一个 RXNE 从未置位的陈旧 RDR —— 陈旧值
           碰巧等于图案字节时会虚报成功。 */
        if (!__HAL_UART_GET_FLAG(&SCS_UART, UART_FLAG_RXNE)) {
            break;                          /* 这一字节没回来 */
        }

        if ((uint8_t)SCS_UART.Instance->RDR == pat[i]) { ok++; }
    }

    return ok;
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

/* ── 字节序 ────────────────────────────────────────────────────────────
 * 飞特官方《通信协议高低字节问题》原文：
 *     「协议中：SCS 系列高字节在前，SMS/STS 低字节在前」
 * 厂商库不会自动区分 —— 字节序是一个进程级全局量 `End`，只由 setEnd() 切换，
 * 在 scslib/SCS.c:50 的 Host2SCS() / SCS2Host() 里生效。
 *
 * 本工程一条 UART5 上**同时挂了两个字节序相反的系列**：
 *     ID 1     STS3032  → SMS_STS 系列 → 低字节在前 → setEnd(0)
 *     ID 2~6   SCS0009  → SCSCL  系列 → 高字节在前 → setEnd(1)
 * 所以**不能只在 SCS_BusInit() 里设一次**，必须每次调用前按系列重新设定。
 *
 * 不设的后果（实测确认）：SCS0009 收到的目标位置是 `发送值 << 8` —— 发 1 实际
 * 是 256（低字节被当成高字节），每 +1 就固定多转约 75°；发 60 得到 15360，远超
 * 量程被钳到上限，舵机因此停在编码器零点跳变处，手推几度就整圈反转回来。
 *
 * 切换点放在互斥锁**之内**，与总线访问受同一把锁保护，不会和另一系列的帧交错。
 */
static void scs_big_endian(void)    { setEnd(1); }   /* SCS/CL 系列：高字节在前 */
static void scs_little_endian(void) { setEnd(0); }   /* SMS/STS、HLS 系列：低字节在前 */

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

/**
 * @brief  取最近一次命令的错误码。
 * @note   这只读 SCS.c 里的静态变量，不占总线，故**刻意不加锁** —— 加锁反而会
 *         读到别的任务刚覆盖的值。
 */
int SCS_GetLastError(void)
{
    return getLastError();
}

/**
 * @brief  声明后续**总线回读**命令面向的是哪一套字节序的舵机。
 * @param  big_endian 1=高字节在前（SCS/CL 系列，如 SCS0009）；0=低字节在前（SMS/STS、HLS）
 * @note   写命令不必调 —— SCS_WritePos() / SCS_WritePosEx() 等系列专属包装会自己设好。
 *         需要它的是回读函数（SCS_ReadPos(id) / SCS_ReadSpeed(id) / …）：这些在厂商库里
 *         是 SCS 与 SMS/STS **共用**的实现，光看 id 分不出系列，只能由调用方声明。
 * @note   只影响「走总线的读」（id >= 0）。id == -1 的缓冲区路径不经过 End。
 */
void SCS_SetEnd(uint8_t big_endian)
{
    setEnd(big_endian ? 1u : 0u);
}

/* --- EPROM 解锁（SMS_STS 系列专用）--- */

int SCS_UnlockEprom(uint8_t id)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = unLockEpromEx(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_LockEprom(uint8_t id)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = LockEpromEx(id);
    if (held) { SCS_Unlock(); }
    return r;
}

/* --- SCSCL 系列 --- */

int SCS_WritePos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
    int held = SCS_Lock();
    scs_big_endian();
    int r = WritePos(id, position, time, speed);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_RegWritePos(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
    int held = SCS_Lock();
    scs_big_endian();
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
    scs_big_endian();
    SyncWritePos(id, idn, position, time, speed);
    if (held) { SCS_Unlock(); }
}

int SCS_EnableTorque(uint8_t id, uint8_t enable)
{
    int held = SCS_Lock();
    scs_big_endian();
    int r = EnableTorque(id, enable);
    if (held) { SCS_Unlock(); }
    return r;
}

/* id 必须是 int: 厂商用 ID==-1 表示「取 FeedBack 缓冲区」而不是上总线。
   ReadPos/ReadMove 在 SCSCL.c 里正是按这个语义实现的。 */
int SCS_ReadPos(int id)
{
    int held = SCS_Lock();
    int r = ReadPos(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_ReadMove(int id)
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
    scs_little_endian();
    int r = WritePosEx(id, position, speed, acc);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_RegWritePosEx(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = RegWritePosEx(id, position, speed, acc);
    if (held) { SCS_Unlock(); }
    return r;
}

void SCS_SyncWritePosEx(uint8_t id[], uint8_t idn, int16_t position[], uint16_t speed[], uint8_t acc[])
{
    int held = SCS_Lock();
    scs_little_endian();
    SyncWritePosEx(id, idn, position, speed, acc);
    if (held) { SCS_Unlock(); }
}

void SCS_SyncWriteSpe(uint8_t id[], uint8_t idn, int16_t speed[], uint8_t acc[])
{
    int held = SCS_Lock();
    scs_little_endian();
    SyncWriteSpe(id, idn, speed, acc);
    if (held) { SCS_Unlock(); }
}

int SCS_WheelMode(uint8_t id)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = WheelMode(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_SetMode(uint8_t id, uint8_t mode)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = SetMode(id, mode);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_WriteSpe(uint8_t id, int16_t speed, uint8_t acc)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = WriteSpe(id, speed, acc);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_CalibrationOfs(uint8_t id)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = CalibrationOfs(id);
    if (held) { SCS_Unlock(); }
    return r;
}

/* --- 状态回读（id 传 -1 表示取 FeedBack 缓冲区，见头文件说明）
 *
 * 注意这些函数在厂商库里的**实现位于 scslib/SCSCL.c**，不在 SMS_STS.c ——
 * 因为 SMS/STS 与 SCS/CL 共用同一张内存表（PRESENT_SPEED_L 等地址完全一致），
 * 厂商就让 STS 系列复用了 SCSCL 的实现。这不是笔误，也不要「修正」到 SMS_STS.c。
 *
 * @warning **这一组函数不自动切换字节序。** 它们两个系列共用，光凭 id 分不出
 *          该用哪套 —— 调用方须先用 SCS_SetEnd() 声明目标舵机属于哪个系列，
 *          否则会沿用上一次的值（写命令留下的，未必是你想要的）。
 *          缓冲区路径（id == -1）则**完全不受 End 影响**，原因见头文件。
 */

int SCS_ReadSpeed(int id)
{
    int held = SCS_Lock();
    int r = ReadSpeed(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_ReadLoad(int id)
{
    int held = SCS_Lock();
    int r = ReadLoad(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_ReadVoltage(int id)
{
    int held = SCS_Lock();
    int r = ReadVoltage(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_ReadTemper(int id)
{
    int held = SCS_Lock();
    int r = ReadTemper(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_ReadCurrent(int id)
{
    int held = SCS_Lock();
    int r = ReadCurrent(id);
    if (held) { SCS_Unlock(); }
    return r;
}

/* --- HLS 系列 --- */

int SCS_WritePosEx2(uint8_t id, int16_t position, uint16_t speed, uint8_t acc, uint16_t torque)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = WritePosEx2(id, position, speed, acc, torque);
    if (held) { SCS_Unlock(); }
    return r;
}

void SCS_SyncWritePosEx2(uint8_t id[], uint8_t idn, int16_t position[], uint16_t speed[], uint8_t acc[], uint16_t torque[])
{
    int held = SCS_Lock();
    scs_little_endian();
    SyncWritePosEx2(id, idn, position, speed, acc, torque);
    if (held) { SCS_Unlock(); }
}

int SCS_WriteSpeEx(uint8_t id, int16_t speed, uint8_t acc, uint16_t torque)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = WriteSpeEx(id, speed, acc, torque);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_EleMode(uint8_t id)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = EleMode(id);
    if (held) { SCS_Unlock(); }
    return r;
}

int SCS_WriteEle(uint8_t id, int16_t torque)
{
    int held = SCS_Lock();
    scs_little_endian();
    int r = WriteEle(id, torque);
    if (held) { SCS_Unlock(); }
    return r;
}
