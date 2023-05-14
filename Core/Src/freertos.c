/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usbd_cdc_if.h"
#include "usart.h"

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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usbSendTask */
osThreadId_t usbSendTaskHandle;
const osThreadAttr_t usbSendTask_attributes = {
  .name = "usbSendTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for modbusSendTask */
osThreadId_t modbusSendTaskHandle;
const osThreadAttr_t modbusSendTask_attributes = {
  .name = "modbusSendTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for USB_queue */
osMessageQueueId_t USB_queueHandle;
const osMessageQueueAttr_t USB_queue_attributes = {
  .name = "USB_queue"
};
/* Definitions for modbus_queue */
osMessageQueueId_t modbus_queueHandle;
const osMessageQueueAttr_t modbus_queue_attributes = {
  .name = "modbus_queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUSBSendTask(void *argument);
void StartModbusSendTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of USB_queue */
  USB_queueHandle = osMessageQueueNew (8, 256, &USB_queue_attributes);

  /* creation of modbus_queue */
  modbus_queueHandle = osMessageQueueNew (8, 256, &modbus_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of usbSendTask */
  usbSendTaskHandle = osThreadNew(StartUSBSendTask, NULL, &usbSendTask_attributes);

  /* creation of modbusSendTask */
  modbusSendTaskHandle = osThreadNew(StartModbusSendTask, NULL, &modbusSendTask_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUSBSendTask */
/**
* @brief Function implementing the usbSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSBSendTask */
void StartUSBSendTask(void *argument)
{
  /* USER CODE BEGIN StartUSBSendTask */
  osStatus_t status;
  modbusPacket modbus_packet;
  uint8_t res;
  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(USB_queueHandle, &modbus_packet, NULL, osWaitForever);
    if ((status == osOK) && (modbus_packet.data_len > 0)){
      res = CDC_Transmit_FS(&modbus_packet.data[0], modbus_packet.data_len);
      if ( res != USBD_OK){
        res = CDC_Transmit_FS(&modbus_packet.data[0], modbus_packet.data_len);
      }
    }
  }
  /* USER CODE END StartUSBSendTask */
}

/* USER CODE BEGIN Header_StartModbusSendTask */
/**
* @brief Function implementing the modbusSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartModbusSendTask */
void StartModbusSendTask(void *argument)
{
  /* USER CODE BEGIN StartModbusSendTask */
  modbusPacket transmit_modbus_packet;
  modbusPacket receive_modbus_packet;
  uint8_t i = 0;
  osStatus_t status;
  uint8_t modbus_byte;
  HAL_StatusTypeDef hal_status_transmit;
  HAL_StatusTypeDef hal_status_receive;
  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(modbus_queueHandle, &transmit_modbus_packet, NULL, 50);
    if (status == osOK){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      if ((hal_status_transmit = HAL_UART_Transmit(&huart2, &transmit_modbus_packet.data[0], transmit_modbus_packet.data_len, 20)) == HAL_OK){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        while ((hal_status_receive = HAL_UART_Receive(&huart2, &modbus_byte, 1, 20)) == HAL_OK){
          receive_modbus_packet.data[i++] = modbus_byte;
        }
        receive_modbus_packet.data_len = i;
        i = 0;
        if ( receive_modbus_packet.data_len > 0){
          status = osMessageQueuePut(USB_queueHandle, &receive_modbus_packet, 0U, 0U);
          if (status == osOK){
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
          }
        }
      }
    }
  }
  /* USER CODE END StartModbusSendTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

