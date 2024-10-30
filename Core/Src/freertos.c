/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "lwip/apps/lwiperf.h"
#include "lwip.h"
#include "usart.h"
#include "fdcan.h"
#include <string.h>
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
volatile uint32_t ulIdleTickCount = 0;
TaskHandle_t xIdleTaskHandle = NULL;
/* USER CODE END Variables */
/* Definitions for DefaultTask */
osThreadId_t DefaultTaskHandle;
const osThreadAttr_t DefaultTask_attributes = {
  .name = "DefaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for Uart1Task */
osThreadId_t Uart1TaskHandle;
const osThreadAttr_t Uart1Task_attributes = {
  .name = "Uart1Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Can1Task */
osThreadId_t Can1TaskHandle;
const osThreadAttr_t Can1Task_attributes = {
  .name = "Can1Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Uart2Task */
osThreadId_t Uart2TaskHandle;
const osThreadAttr_t Uart2Task_attributes = {
  .name = "Uart2Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Uart1Queue */
osMessageQueueId_t Uart1QueueHandle;
const osMessageQueueAttr_t Uart1Queue_attributes = {
  .name = "Uart1Queue"
};
/* Definitions for Can1Queue */
osMessageQueueId_t Can1QueueHandle;
const osMessageQueueAttr_t Can1Queue_attributes = {
  .name = "Can1Queue"
};
/* Definitions for Uart2Queue */
osMessageQueueId_t Uart2QueueHandle;
const osMessageQueueAttr_t Uart2Queue_attributes = {
  .name = "Uart2Queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vCalculateCPUUsage() {
    static uint32_t ulLastIdleTickCount = 0;
    uint32_t ulCurrentIdleTickCount = ulIdleTickCount;
    uint32_t ulIdleTicks = ulCurrentIdleTickCount - ulLastIdleTickCount;
    ulLastIdleTickCount = ulCurrentIdleTickCount;

    uint32_t ulTotalTicks = configTICK_RATE_HZ;
    uint32_t ulCPUUsage = 100 - (ulIdleTicks * 100 / ulTotalTicks);
	printf("CPU Usage: %lu%%\n", ulCPUUsage);
}

void vCalculateRAMUsage() {
	TaskHandle_t xHandle = xTaskGetHandle("tcpip_thread");
	UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(xHandle);
	printf("Stack Usage: %lu\n", uxHighWaterMark);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartUart1Task(void *argument);
void StartCan1Task(void *argument);
void StartUart2Task(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
    if( xIdleTaskHandle == NULL )
    {
        /* Store the handle to the idle task. */
    	xIdleTaskHandle = xTaskGetCurrentTaskHandle();
    }
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
	if (xTaskGetCurrentTaskHandle() == xIdleTaskHandle) {
		ulIdleTickCount++;
	}
}
/* USER CODE END 3 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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

  /* Create the queue(s) */
  /* creation of Uart1Queue */
  Uart1QueueHandle = osMessageQueueNew (1024, sizeof(uint8_t), &Uart1Queue_attributes);

  /* creation of Can1Queue */
  Can1QueueHandle = osMessageQueueNew (8, sizeof(can_rx_msg_t), &Can1Queue_attributes);

  /* creation of Uart2Queue */
  Uart2QueueHandle = osMessageQueueNew (1024, sizeof(uint8_t), &Uart2Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DefaultTask */
  DefaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &DefaultTask_attributes);

  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

  /* creation of Uart1Task */
  Uart1TaskHandle = osThreadNew(StartUart1Task, NULL, &Uart1Task_attributes);

  /* creation of Can1Task */
  Can1TaskHandle = osThreadNew(StartCan1Task, NULL, &Can1Task_attributes);

  /* creation of Uart2Task */
  Uart2TaskHandle = osThreadNew(StartUart2Task, NULL, &Uart2Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  LOCK_TCPIP_CORE();
  lwiperf_start_tcp_server_default(NULL, NULL);
  UNLOCK_TCPIP_CORE();
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
	vCalculateCPUUsage();
	vCalculateRAMUsage();
	osDelay(1000);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartUart1Task */
/**
* @brief Function implementing the Uart1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart1Task */
void StartUart1Task(void *argument)
{
  /* USER CODE BEGIN StartUart1Task */
  uint8_t c;
  /* Infinite loop */
  for(;;)
  {
	if (osMessageQueueGet(Uart1QueueHandle, &c, NULL, osWaitForever) == osOK) {
	  HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
	}
	osDelay(1);
  }
  /* USER CODE END StartUart1Task */
}

/* USER CODE BEGIN Header_StartCan1Task */
/**
* @brief Function implementing the Can1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCan1Task */
void StartCan1Task(void *argument)
{
  /* USER CODE BEGIN StartCan1Task */
  CAN_Filter_Config();
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan1);
  /* Infinite loop */
  for(;;)
  {
	if (osMessageQueueGet(Can1QueueHandle, &can_rx_msg, NULL, osWaitForever) == osOK) {
	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_msg.txHeader, can_rx_msg.data);
	}
	osDelay(1);
  }
  /* USER CODE END StartCan1Task */
}

/* USER CODE BEGIN Header_StartUart2Task */
/**
* @brief Function implementing the Uart2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart2Task */
void StartUart2Task(void *argument)
{
  /* USER CODE BEGIN StartUart2Task */
  uint8_t c;
  /* Infinite loop */
  for(;;)
  {
	if (osMessageQueueGet(Uart2QueueHandle, &c, NULL, 0) == osOK) {
	  HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
	}
	osDelay(1);
  }
  /* USER CODE END StartUart2Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if (hfdcan->Instance == FDCAN1 && (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
	  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can_rx_msg.rxHeader, can_rx_msg.data) == HAL_OK) {
		  osMessageQueuePut(Can1QueueHandle, &can_rx_msg, 0, 0);
	  }
  }
}

int __io_putchar(int ch) {
    char c = (char)ch;
    osMessageQueuePut(Uart1QueueHandle, &c, 0, 0);
    osMessageQueuePut(Uart2QueueHandle, &c, 0, 0);
    return ch;
}
/* USER CODE END Application */

