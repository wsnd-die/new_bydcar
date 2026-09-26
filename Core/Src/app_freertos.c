/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Common_used.h"    /* 工程聚合头: FreeRTOS / HAL / 各业务模块 */
#include "worker_task.h"    /* FC_Task / NLF_Task / NLF_Request */
#include "emm_5v.h"
#include "ops9_g491_uart3.h"
#include "servo_scs.h"
#include "worker_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── 总线上的 6 个舵机 (UART5) ────────────────────────────────────────
 * @note 这几个 #define 必须放在 USER CODE 区里 —— 放到 gripper_task 上方那一段
 *       (USER CODE END Header_gripper_task 与函数签名之间) 属于生成区，
 *       CubeMX 重新生成会**静默删掉**。
 *
 * @warning **ID 1 与 ID 2~6 属于两个不同的系列，必须用两套不同的 API 驱动。**
 *          这不是代码风格问题，是寄存器布局问题：
 *
 *          ID 1  STS3032  → SMS_STS 系列
 *              起始地址 41(ACC)，一次写 7 字节 [ACC|位置|时间|速度]
 *              位置量程 **0~4095** 对应 0~360°，中位 **2048**
 *              → `SCS_WritePosEx(id, pos, speed, acc)`
 *
 *          ID 2~6 SCS0009 → SCSCL 系列（厂商型号表 `5,4,4,1,SCS009`）
 *              起始地址 42(GOAL_POSITION)，一次写 6 字节 [位置|时间|速度]
 *              位置量程 **0~1000** 对应 0~300°，中位 **500**
 *              → `SCS_WritePos(id, pos, time, speed)`
 *
 *          把 SCS0009 交给 `SCS_WritePosEx()` 的后果：ACC 字节会落到 SCSCL 未
 *          定义的 41 号地址上，位置还会超出 0~1000 的量程。**不会报错，只是不动。**
 *          详见 servo_scs.h 各自接口的 @note。 */
#define SERVO_ID_STS3032      1      /* STS3032, SMS_STS 系列 */
#define SERVO_ID_SCS0009_MIN  2      /* SCS0009, SCSCL 系列 */
#define SERVO_ID_SCS0009_MAX  6


/* 两套量程各自的参数。中位与速度单位都不是同一套，别互相抄。
 *
 * @warning **这两个 CENTER 必须取各自量程的中段，绝不能贴住量程两端。**
 *          位置寄存器是单圈绝对值 —— 0 与量程上限在物理上是**相邻的同一个点**。
 *          目标位置停在那里时，手推几度就会让读数从一端跳到另一端，位置环把
 *          误差算成「差一整圈」，于是顺着你推的方向转满一圈才回来。
 *          取 0 / 2 / 4095 这类值，即使字节序修好了，该现象**依然会出现**。 */
#define STS_CENTER     2048    /* STS3032: 0~4095 的中位 */
#define STS_SPEED       0    /* 原始寄存器值，单位见 STS3032 数据手册 */
#define STS_ACC         0      /* 原始寄存器值，0 = 不控加速度直冲最高速 */
#define SCS_CENTER     450     /* SCS0009: 0~1024 的中位（0.293°/步，全行程 300°） */
#define SCS_SPEED       0    /* 原始寄存器值，0 = 用寄存器内部值 */
#define SCS_TIME        0       /* 0 = 用寄存器内部值 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Worker 任务属性。句柄 (fcTaskHandle / nlfTaskHandle) **不在本文件定义** ——
 * 它们定义在 app/worker_task.c:38-39, 因为调度器要通过句柄给任务发线程标志。
 * 本文件只负责创建并把句柄赋值回去。
 *
 * @note 这两组属性必须放在 USER CODE 区。上面生成区那三组 (defaultTask /
 *       ops9imu_task / gripper) 由 CubeMX 按 .ioc 的 FREERTOS.Tasks01 生成,
 *       手工加的内容会在重新生成时被静默删掉。
 *
 * 优先级: ops9imu_task = osPriorityHigh(40) 保持最高; FC_TASK 取 AboveNormal(32),
 *         高于 NLF_TASK 的 Normal(24) —— FC_TASK 是 10ms 角度环, 必须能抢占
 *         阻塞式流程任务, 否则控制周期会被拉长。
 *
 * 栈深: 直接复用 app/worker_task.h:58-59 的宏, 不写字面量 (规范第 7.3 节)。
 *       CMSIS-RTOS2 的 stack_size 单位是**字节**, 故乘 4。 */
const osThreadAttr_t fcTask_attributes = {
  .name       = "FC_TASK",
  .priority   = (osPriority_t) osPriorityAboveNormal,
  .stack_size = FC_TASK_STACK_WORDS * 4
};

const osThreadAttr_t nlfTask_attributes = {
  .name       = "NLF_TASK",
  .priority   = (osPriority_t) osPriorityNormal,
  .stack_size = NLF_TASK_STACK_WORDS * 4
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for ops9imu_task */
osThreadId_t ops9imu_taskHandle;
const osThreadAttr_t ops9imu_task_attributes = {
  .name = "ops9imu_task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for gripper */
osThreadId_t gripperHandle;
const osThreadAttr_t gripper_attributes = {
  .name = "gripper",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for findcircle_TASK */
osThreadId_t findcircle_TASKHandle;
const osThreadAttr_t findcircle_TASK_attributes = {
  .name = "findcircle_TASK",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for nav_task */
osThreadId_t nav_taskHandle;
const osThreadAttr_t nav_task_attributes = {
  .name = "nav_task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void servo_set_pos(uint8_t id, uint16_t pos);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ops9imu_fuction(void *argument);
void gripper_task(void *argument);
void FC_TASK(void *argument);
void NLF_TASK(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* 建系统事件队列。必须在创建任何任务之前 —— 调度器的 task_recive()
   * 依赖 systemEventQueue, 而它由本函数创建 (此前从未被调用, 故为 NULL)。 */
  task_init();


  /* 注意: 这里**不能**发 CAN 命令。本函数在 osKernelStart() 之前执行, 而此刻
   * pxCurrentTCB 仍是 NULL (tasks.c:337 初值, 直到第一个任务被创建才在
   * prvAddNewTaskToDelayedList 里赋值)。can_SendCmd() → FDCAN_WaitFreeTxFifo()
   * 在 TX FIFO 满时会调 osDelay(1), 而 osDelay 在 CMSIS-RTOS2 里对"调度器未启动"
   * 没有任何保护, 会一路走到 vTaskDelay → prvAddCurrentTaskToDelayedList →
   * uxListRemove(&(pxCurrentTCB->xStateListItem)) 直接 NULL 解引用 → HardFault。
   * 使能命令改放到 StartDefaultTask 里发。 */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ops9imu_task */
  ops9imu_taskHandle = osThreadNew(ops9imu_fuction, NULL, &ops9imu_task_attributes);

  /* creation of gripper */
  gripperHandle = osThreadNew(gripper_task, NULL, &gripper_attributes);

  /* creation of findcircle_TASK */
  findcircle_TASKHandle = osThreadNew(FC_TASK, NULL, &findcircle_TASK_attributes);

  /* creation of nav_task */
  nav_taskHandle = osThreadNew(NLF_TASK, NULL, &nav_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   /* 保留原有上电动作 */



  /* ── 调度器 ──────────────────────────────────────────────────────────
   * 架构: 驱动源 → defaultTask 调度器 → Worker 任务。
   * 本任务只负责从系统事件队列取 Mode 并转交 NLF_TASK, 不再碰 IMU ——
   * HWT906 的轮询已交还 FC_TASK (见 app/worker_task.c 的 FC_Task)。
   *
   * task_recive() 内部是 portMAX_DELAY 阻塞, 队列空时本任务挂起、不占 CPU;
   * 下面的 osDelay(20) 只在真的收到一条命令之后才会执行。 */
  for(;;)
  {
    TaskCommand_t cmd = task_recive();
    if (cmd.k) {
      NLF_Request(cmd.Mode);
    }

    // Emm_V5_Vel_Control(1, 0, 0, 0, 0);
    // Emm_V5_Vel_Control(2, 0, 0, 0, 0);
    // Emm_V5_Vel_Control(3, 1, 0, 0, 0);
    // Emm_V5_Vel_Control(4, 1, 0, 0, 0);

    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ops9imu_fuction */
/**
* @brief Function implementing the ops9imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ops9imu_fuction */
void ops9imu_fuction(void *argument)
{
  /* USER CODE BEGIN ops9imu_fuction */
  /* Infinite loop */
  const LocatorDev_t *active_locator = &locator_ops9;
  PoseData_t o_pose;

  /* Infinite loop */
  active_locator->init();
  for(;;)
  {
    active_locator->update();
    active_locator->get_pose(&o_pose);
    // printf("xyyaw:%f,%f,%f\r\n",o_pose.x,o_pose.y,o_pose.yaw);
    osDelay(6);
  }
  /* USER CODE END ops9imu_fuction */
}

/* USER CODE BEGIN Header_gripper_task */
/**
* @brief Function implementing the gripper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gripper_task */
void gripper_task(void *argument)
{
  /* USER CODE BEGIN gripper_task */
  /* ── 总线初始化 ────────────────────────────────────────────────────
   * 串口助手接 huart2 (PA2/PA3, 115200) 看 printf 输出。
   * 常量见本文件 USER CODE BEGIN PD 区。 */
  if (!SCS_BusInit()) {
    /* huart5.Instance == NULL —— MX_UART5_Init() 没跑，见变更记录 V1.10.0 备注 2 */
    printf("[scs] bus init FAIL: huart5 not initialized\r\n");
    for (;;) { osDelay(100); }
  }

  servo_set_pos(SERVO_ID_STS3032, STS_CENTER);
  for (uint8_t id = SERVO_ID_SCS0009_MIN; id <= SERVO_ID_SCS0009_MAX; id++) {
    servo_set_pos(id, SCS_CENTER);
  }


  for (;;)
  {
    /* ── 在这里写你的舵机控制逻辑 ────────────────────────────────────
     * 用 servo_set_pos() 就行，它会按 ID 自动分派到正确的系列上：
     *
     *     servo_set_pos(1, 1000);   // STS3032 → 量程 0~4095
     *     servo_set_pos(3, 700);    // SCS0009 → 量程 0~1000
     *
     * 需要分别控制速度/时间/加速度时，直接调底层接口（注意量程与单位不同）：
     *
     *     SCS_WritePosEx(1, pos, speed, acc);   // 仅 STS3032
     *     SCS_WritePos(2, pos, time, speed);    // 仅 SCS0009
     *
     * 回读用 FeedBack 一次取全，再 ReadXxx(-1) 从缓冲区拿，不额外占总线：
     *
     *     SCS_FeedBack(1);
     *     if (SCS_GetLastError() == 0) { int p = SCS_ReadPos(-1); }
     */
    osDelay(20);
  }
  /* USER CODE END gripper_task */
}

/* USER CODE BEGIN Header_FC_TASK */
/**
* @brief Function implementing the findcircle_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FC_TASK */
void FC_TASK(void *argument)
{
  /* USER CODE BEGIN FC_TASK */
  /* Infinite loop */
  for(;;)
  {
    FC_Fuction();
    osDelay(1);
  }
  /* USER CODE END FC_TASK */
}

/* USER CODE BEGIN Header_NLF_TASK */
/**
* @brief Function implementing the nav_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NLF_TASK */
void NLF_TASK(void *argument)
{
  /* USER CODE BEGIN NLF_TASK */
  /* Infinite loop */
  for(;;)
  {
    NLF_Fuction();
    osDelay(1);
  }
  /* USER CODE END NLF_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  按系列分派的位置控制 —— 上层不必记住哪个 ID 是哪个系列。
  * @param  id   舵机 ID：1 = STS3032(SMS_STS 系列)，2~6 = SCS0009(SCSCL 系列)
  * @param  pos  目标位置。**量程由系列决定，两者不能混用**：
  *              STS3032 是 0~4095（中位 2048），SCS0009 是 0~1000（中位 500）。
  * @note   速度/加速度取 PD 区里各自系列的常量，本函数不暴露这几个参数；
  *         需要单独调速时直接调 SCS_WritePosEx() / SCS_WritePos()。
  * @note   两个系列不能互换 API：把 SCS0009 交给 SCS_WritePosEx() 会把 ACC
  *         字节写到 SCSCL 未定义的 41 号地址，且位置超出 0~1000 量程 ——
  *         不报错，只是不动。原因见本文件 PD 区的说明。
  */
static void servo_set_pos(uint8_t id, uint16_t pos)
{
  if (id == SERVO_ID_STS3032) {
    (void)SCS_WritePosEx(id, (int16_t)pos, STS_SPEED, STS_ACC);
  } else {
    (void)SCS_WritePos(id, pos, SCS_TIME, SCS_SPEED);
  }
}

/**
  * @brief  栈溢出钩子 (configCHECK_FOR_STACK_OVERFLOW = 2 时由内核调用)。
  * @param  xTask       溢出任务的句柄
  * @param  pcTaskName  任务名
  * @note   这里刻意停住而不是复位: 用调试器可以直接看到是哪条任务、栈用了多少。
  *         比"随机跳 HardFault"可诊断得多。历史上本工程的栈溢出是完全静默的。
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;

  taskDISABLE_INTERRUPTS();
  /* 留一个可断点的位置: 断在这里, 看 pcTaskName 就知道是谁溢出了。 */
  for (;;)
  {
    __NOP();
  }
}

/* USER CODE END Application */

