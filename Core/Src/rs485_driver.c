#include "main.h"
#include "board.h"
#include "usart.h"
#include "rs485_driver.h"
#include "cmsis_os.h"

uint8_t rs485_transmit(const dataPacket * const packet){
  HAL_StatusTypeDef transmit_status;
  LED_TX_On();
  RS485_TX();
  if ((transmit_status = HAL_UART_Transmit(&huart2, &packet->data[0], packet->data_len, 20)) != HAL_OK){
    return 1;
  }
  RS485_RX();
  LED_TX_Off();
  return 0;
}

uint8_t rs485_receive(dataPacket *packet){
  HAL_StatusTypeDef receive_status;
  osStatus_t status;
  uint8_t i = 0;
  uint8_t modbus_byte;
  LED_RX_On();
  RS485_RX();
  while ((receive_status = HAL_UART_Receive(&huart2, &modbus_byte, 1, 20)) == HAL_OK){
    packet->data[i] = modbus_byte;
    i++;
  }
  LED_RX_Off();
  packet->data_len = i;
  if (packet->data_len > 0){
    return 0;
  }else{
    return 1;
  }
}
