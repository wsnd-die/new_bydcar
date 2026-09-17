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
const osThreadAttr_t fcTask_attributes = {
  .name = "FC_TASK",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = FC_TASK_STACK_WORDS * 4
};

/* NLF_TASK: 流程任务, 大部分时间阻塞在导航/循迹里 */
const osThreadAttr_t nlfTask_attributes = {
  .name = "NLF_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = NLF_TASK_STACK_WORDS * 4
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* Worker 任务。架构: 驱动源 → defaultTask 调度器 → Worker 任务,
   * 见 app/banyuntask.h 与 app/worker_task.h。 */
  fcTaskHandle  = osThreadNew(FC_Task,  NULL, &fcTask_attributes);
  nlfTaskHandle = osThreadNew(NLF_Task, NULL, &nlfTask_attributes);
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
  /* 事件调度器: 阻塞等事件, 按 Mode 转交 Worker 任务执行。
   *
   * 本任务刻意不承担任何阻塞式工作 —— 它一旦跑去导航, 就收不到后续事件了。
   * 真正的执行体在 NLF_TASK (app/worker_task.c)。
   *
   * 原实现是个 osDelay(1) 空循环, 从未调用过 task_recive(); 配合 task_init()
   * 从未被调用, 整个事件队列机制在此之前是空转的。 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);   /* 保留原有上电动作 */

  for(;;)
  {
    TaskCommand_t cmd = task_recive();
    if (cmd.k) {
      NLF_Request(cmd.Mode);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

