#ifndef BOARD_CONF_H_
#define BOARD_CONF_H_

#include "stm32f1xx_hal.h"

#define RS485_SPEED 9600

typedef enum {
    USB2RS485,
    USB2CAN,
}converter_mode;

typedef struct{
  uint8_t data_len;
  uint8_t data[256];
} dataPacket;

#define LED_RUN_On()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET)
#define LED_RUN_Off()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)

#define LED_RX_On()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define LED_RX_Off()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)

#define LED_TX_On()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define LED_TX_Off()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

#define RS485_RX()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define RS485_TX()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)



#endif /* BOARD_CONF_H_ */
