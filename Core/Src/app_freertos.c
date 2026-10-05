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
#include "NX_uart.h"
#include "block_basic.h"
#include "mecanum.h"
#include "Send_motor.h"
#include "worker_task.h"
#include "key.h"
#include "NavigationMecanum.h"
#include "collect_ir.h"
#include "block_collect.h"  /* BlockCollect_Task (V1.20.0) */
#include "msp_color.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Worker 任务的句柄 (fcTaskHandle / nlfTaskHandle) **不在本文件定义** ——
 * 它们定义在 app/worker_task.c, 因为 NLF_TASK 要拿自己的句柄做
 * osThreadFlagsSet 自唤醒 (见 worker_task.c 的 NLF_Request)。
 * 本文件只负责创建任务、并把生成的句柄转存过去, 见 MX_FREERTOS_Init 里
 * USER CODE BEGIN RTOS_THREADS 那一段。
 *
 * @note 任务的属性 (优先级/栈深/名字) **全部由 .ioc 的 FREERTOS.Tasks01 生成**,
 *       本文件不再另存一份 —— 此前这里曾有两组重复的 fcTask_attributes /
 *       nlfTask_attributes, 定义了却从没传给 osThreadNew, 是纯死代码,
 *       2026-09-28 清理掉。要改栈深请改 .ioc, 否则重新生成会被冲掉。
 *
 * 名字: 两个任务在 .ioc 里的名字是 FC_TASK / NLF_TASK, 与入口函数同名。
 *       此前叫 findcircle_TASK / nav_task (入口却是 FC_TASK / NLF_TASK),
 *       是纯误导, 已改正。 */

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
/* Definitions for FC_TASK */
osThreadId_t FC_TASKHandle;
const osThreadAttr_t FC_TASK_attributes = {
  .name = "FC_TASK",
  .priority = (osPriority_t) osPriorityNormal3,
  .stack_size = 256 * 4
};
/* Definitions for NLF_TASK */
osThreadId_t NLF_TASKHandle;
const osThreadAttr_t NLF_TASK_attributes = {
  .name = "NLF_TASK",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for angle_Task */
osThreadId_t angle_TaskHandle;
const osThreadAttr_t angle_Task_attributes = {
  .name = "angle_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for key_Task */
osThreadId_t key_TaskHandle;
const osThreadAttr_t key_Task_attributes = {
  .name = "key_Task",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
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
void AC_Fuction(void *argument);
void KEY_TASK(void *argument);

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
  Key_Init();
  IR_Init();

  /* MSP 颜色芯片: 挂上 USART2 的 DMA-IDLE 接收 (V1.20.6)。
   * 放在这里是因为 USART2 已在 MX_USART2_UART_Init() 里配好、DMA 句柄也已
   * 由它的 MspInit 链接好, 而本函数在 osKernelStart() 之前跑。
   * **全工程只此一处** —— 重复调用会先 memset 再重挂, 而 DMA 已经挂着,
   * HAL_UARTEx_ReceiveToIdle_DMA 返回 HAL_BUSY 直接退出, 结果是状态被清、
   * 接收却没重挂。 */
  MSP_Color_Init();
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

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

  /* creation of FC_TASK */
  FC_TASKHandle = osThreadNew(FC_TASK, NULL, &FC_TASK_attributes);

  /* creation of NLF_TASK */
  NLF_TASKHandle = osThreadNew(NLF_TASK, NULL, &NLF_TASK_attributes);

  /* creation of angle_Task */
  angle_TaskHandle = osThreadNew(AC_Fuction, NULL, &angle_Task_attributes);

  /* creation of key_Task */
  key_TaskHandle = osThreadNew(KEY_TASK, NULL, &key_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* Worker 任务句柄转存 —— app/worker_task.c 的 NLF_Request() 要用
   * nlfTaskHandle 给 NLF_TASK 发线程标志。
   *
   * @warning 这**不是**可选的: 此前 worker_task.c 里那两个句柄初值 NULL 且
   *          全工程无写入, 使 `if (nlfTaskHandle != NULL)` 恒假 →
   *          osThreadFlagsSet() 一次都没执行过 → NLF_TASK 永远阻塞在
   *          osThreadFlagsWait, 整条流程一次也没跑起来 (2026-09-28 修)。
   *
   * 必须放在 USER CODE 区: 上面那些 osThreadNew 是 CubeMX 按 .ioc 生成的,
   * 重新生成会覆盖生成区, 但保留本区。 */
  nlfTaskHandle = NLF_TASKHandle;
  fcTaskHandle  = FC_TASKHandle;

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

  Emm_V5_En_Control(1,1,0);
  Emm_V5_En_Control(2,1,0);
  Emm_V5_En_Control(3,1,0);
  Emm_V5_En_Control(4,1,0);
  Emm_V5_En_Control(5,1,0);
  // osDelay(100);
  // for (;;) {
  //   Emm_V5_Vel_Control(1, 0, 50, 0, 0);
  //   Emm_V5_Vel_Control(2, 0, 50, 0, 0);
  //   Emm_V5_Vel_Control(3, 1, 50, 0, 0);
  //   Emm_V5_Vel_Control(4, 1, 50, 0, 0);
  //
  //   osDelay(20);
  //   Emm_V5_Synchronous_motion(0);
  //   Print_can_error_count();
  // }

  // BlockBasic_LiftTo(UP,20);
  // Emm_V5_Pos_Control(5, 1, 800, 255, 32000, 0, 0);
  // PoseData_t p0;
  // osDelay(2000);
  //   locator_ops9.get_pose(&p0);
  //   printf("[NAV] 起点 (%.3f, %.3f, %.1f deg)\r\n",
  //          p0.x, p0.y, p0.yaw * 57.29578f);
  //
  //   bool ok = Nav_GoToWorld(p0.x - 1.30f, p0.y-0.5, p0.yaw-0);
  //
  //   printf("[NAV] ok=%d, Self_Dir=(%.3f, %.3f, %.1f deg)\r\n",
  //          (int)ok, Self_Dir.x, Self_Dir.y, Self_Dir.yaw * 57.29578f);
  for(;;)
  {
    TaskCommand_t cmd = task_recive();
    if (cmd.k) {

    }
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
  //OPS9_G491_UART3_SetPose(2.084f, -257.49f, 1723.84f);
  //OPS9_G491_UART3_SetPose(2.084f, -257.49f, 1723.84f);
  for(;;)
  {
    active_locator->update();
    active_locator->get_pose(&o_pose);
    // if (Key_WasLongPressed(KEY_START) && !BlockCollect_IsRunning() ) {
    //   printf("xyyaw:%f,%f,%f\r\n",o_pose.x,o_pose.y,o_pose.yaw);
    //
    // }
   // printf("xyyaw:%f,%f,%f\r\n",o_pose.x,o_pose.y,o_pose.yaw);
    osDelay(5);
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
  if (!SCS_BusInit()) {
    printf("[scs] bus init FAIL: huart5 not initialized\r\n");
    for (;;) { osDelay(100); }
  }
  while (!BPlace_SetZero());
  BlockBasic_LiftSync(0.0f);      /* 压限位归零 → 软件高度同步成 0 (之后用绝对高度指令) */
  printf("[scs] init SUCSESS: huart5 initialized\r\n");
  BlockBasic_TurntableTo(1);
  Servo_SetAngle(127);
  for (uint8_t id = SERVO_ID_SCS0009_MIN; id <= SERVO_ID_SCS0009_MAX; id++) {
    servo_set_pos(id, SCS_OPEN);
  }
  // Servo_Angle(BLOCK_TURNTABLE_HOME_DEG);
  BlockBasic_LiftToAbs(5.0f);     /* 夹爪初始化抬到 3mm (等价原 UP,3) */
  printf("[scs] gripper init done\r\n");
  for (;;)
  {
    BlockCollect_Poll();

    // MSP_Color_DebugPoll();
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
  NX_Init();
  /* Infinite loop */
  for(;;)
  {
    // FC_Fuction();
    osDelay(10);
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
 // Nav_GoToWorld(1.0f,0,0);
  for(;;)
  {
  NLF_Fuction();
    // Nav_GoToWorld(0,0,0);
    osDelay(10);
  }
  /* USER CODE END NLF_TASK */
}

/* USER CODE BEGIN Header_AC_Fuction */
/**
* @brief Function implementing the AC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AC_Fuction */
void AC_Fuction(void *argument)
{
  /* USER CODE BEGIN AC_Fuction */
  /* Infinite loop */
  // MecanumResult cmd = Mecanum_Calc(0.2,0);
  for(;;)
  {
    Angle_Fuction();
    osDelay(5);
    // Send_commandmotor(&cmd);
  }
  /* USER CODE END AC_Fuction */
}

/* USER CODE BEGIN Header_KEY_TASK */
/**
* @brief Function implementing the key_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KEY_TASK */
void KEY_TASK(void *argument)
{
  /* USER CODE BEGIN KEY_TASK */
uint8_t key_StartOK=0;

  for(;;)
  {
    Key_Update();

    /* 用 Key_WasPressed (边沿, 读后清) 而不是 Key_IsPressed (电平):
     * 后者只要按键按着就恒真, 每 10ms 触发一次, 每圈都把流程拽回中继站。 */
    if (Key_WasReleased(KEY_START) && !BlockCollect_IsRunning() && !key_StartOK)
    {
      printf("[KEY] 启动键 -> NLF_Request(Event_Collect_L)\r\n");
      NLF_Request(Event_START);
      //NLF_Request(Event_STARTSecnd);
      Servo_SetAngle(40);
      key_StartOK=1;
    }

    osDelay(10);
  }
  /* USER CODE END KEY_TASK */
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

