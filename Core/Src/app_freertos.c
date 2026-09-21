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

/* Worker 任务属性。句柄 (fcTaskHandle / nlfTaskHandle) 定义在 app/worker_task.c,
 * 因为调度器要通过它们给任务发线程标志。 */

/* FC_TASK: 10ms 角度环, 必须能抢占阻塞式流程任务, 否则控制周期会被拉长 */


/* NLF_TASK: 流程任务, 大部分时间阻塞在导航/循迹里 */

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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ops9imu_fuction(void *argument);

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* Worker 任务。架构: 驱动源 → defaultTask 调度器 → Worker 任务,
   * 见 app/banyuntask.h 与 app/worker_task.h。 */
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


  Emm_V5_En_Control(1, 1, 0);
  Emm_V5_En_Control(2, 1, 0);
  Emm_V5_En_Control(3, 1, 0);
  Emm_V5_En_Control(4, 1, 0);

  for(;;)
  {
    // TaskCommand_t cmd = task_recive();
    // if (cmd.k) {
    //   NLF_Request(cmd.Mode);
    // }

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
  active_locator->init();
  for(;;)
  {
    active_locator->update();
    active_locator->get_pose(&o_pose);
    printf("xyyaw:%f,%f,%f\r\n",o_pose.x,o_pose.y,o_pose.yaw);
    osDelay(10);
  }
  /* USER CODE END ops9imu_fuction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

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

