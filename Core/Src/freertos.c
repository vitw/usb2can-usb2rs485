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
#include "rs485_driver.h"
#include "can_driver.h"
#include "board.h"

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
converter_mode mode;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usb_tx_Task */
osThreadId_t usb_tx_TaskHandle;
const osThreadAttr_t usb_tx_Task_attributes = {
  .name = "usb_tx_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for rs485_tx_Task */
osThreadId_t rs485_tx_TaskHandle;
const osThreadAttr_t rs485_tx_Task_attributes = {
  .name = "rs485_tx_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for can_tx_Task */
osThreadId_t can_tx_TaskHandle;
const osThreadAttr_t can_tx_Task_attributes = {
  .name = "can_tx_Task",
  .stack_size = 256 * 8,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for usb_tx_queue */
osMessageQueueId_t usb_tx_queueHandle;
const osMessageQueueAttr_t usb_tx_queue_attributes = {
  .name = "usb_tx_queue"
};
/* Definitions for rs485_tx_queue */
osMessageQueueId_t rs485_tx_queueHandle;
const osMessageQueueAttr_t rs485_tx_queue_attributes = {
  .name = "rs485_tx_queue"
};
/* Definitions for can_tx_queue */
osMessageQueueId_t can_tx_queueHandle;
const osMessageQueueAttr_t can_tx_queue_attributes = {
  .name = "can_tx_queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void usb_transmit();

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUsbTxTask(void *argument);
void StartRs485TxTask(void *argument);
void StartCanTxTask(void *argument);

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
  /* creation of usb_tx_queue */
  usb_tx_queueHandle = osMessageQueueNew (8, 256, &usb_tx_queue_attributes);

  /* creation of rs485_tx_queue */
  rs485_tx_queueHandle = osMessageQueueNew (8, 256, &rs485_tx_queue_attributes);

  /* creation of can_tx_queue */
  can_tx_queueHandle = osMessageQueueNew (8, 256, &can_tx_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of usb_tx_Task */
  usb_tx_TaskHandle = osThreadNew(StartUsbTxTask, NULL, &usb_tx_Task_attributes);

  /* creation of rs485_tx_Task */
  rs485_tx_TaskHandle = osThreadNew(StartRs485TxTask, NULL, &rs485_tx_Task_attributes);

  /* creation of can_tx_Task */
  can_tx_TaskHandle = osThreadNew(StartCanTxTask, NULL, &can_tx_Task_attributes);

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
    if (mode == USB2CAN) {
      LED_RUN_On();
    }else{
      LED_RUN_Off();
    }
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUsbTxTask */
/**
* @brief Function implementing the usb_tx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbTxTask */
void StartUsbTxTask(void *argument)
{
  /* USER CODE BEGIN StartUsbTxTask */
  osStatus_t status;
  dataPacket modbus_packet;
  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(usb_tx_queueHandle, &modbus_packet, NULL, osWaitForever);
    if ((status == osOK) && (modbus_packet.data_len > 0)){
      usb_transmit(&modbus_packet);
    }
  }
  /* USER CODE END StartUsbTxTask */
}

/* USER CODE BEGIN Header_StartRs485TxTask */
/**
* @brief Function implementing the rs485_tx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRs485TxTask */
void StartRs485TxTask(void *argument)
{
  /* USER CODE BEGIN StartRs485TxTask */
  dataPacket transmit_modbus_packet;
  dataPacket receive_modbus_packet;
  osStatus_t status;
  uint8_t res;
  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(rs485_tx_queueHandle, &transmit_modbus_packet, NULL, 50);
    if (mode == USB2RS485 && status == osOK){
      res = rs485_transmit(&transmit_modbus_packet);
      if (res > 0){
        continue;
      }
      res = rs485_receive(&receive_modbus_packet);
      if (res > 0){
        continue;
      }
      osMessageQueuePut(usb_tx_queueHandle, &receive_modbus_packet, 0U, 0U);
    }
  }
  /* USER CODE END StartRs485TxTask */
}

/* USER CODE BEGIN Header_StartCanTxTask */
/**
* @brief Function implementing the can_tx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTxTask */
void StartCanTxTask(void *argument)
{
  /* USER CODE BEGIN StartCanTxTask */
  dataPacket transmit_cmd;
  osStatus_t status;
  uint8_t res;
  /* Infinite loop */
  for(;;)
  {
    status = osMessageQueueGet(can_tx_queueHandle, &transmit_cmd, NULL, 50);
    if (mode == USB2CAN && status == osOK){
      res = can_transmit(&transmit_cmd);
      if (res > 0){
        continue;
      }
      // res = can_receive(&receive_modbus_packet);
      // if (res > 0){
      //   continue;
      // }
      // osMessageQueuePut(usb_tx_queueHandle, &receive_modbus_packet, 0U, 0U);
    }
  }
  /* USER CODE END StartCanTxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void usb_transmit(dataPacket * packet){
  while(CDC_Transmit_FS(&packet->data[0], packet->data_len) != USBD_OK){
    ;
  }
}
/* USER CODE END Application */

