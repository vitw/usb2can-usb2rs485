#ifndef CAN_DRIVER_H_
#define CAN_DRIVER_H_

typedef struct{
  uint8_t cmd_type;
  uint8_t value[128];
}slcanCmd;

typedef enum {
    SET_BIT_RATE,
    OPEN_CHANNEL,
    CLOSE_CHANNEL,
    TRANSMIT_STANDART_ID,
    GET_VERSION,
    END_CMD,
}slcanCmdType;

uint8_t can_transmit(dataPacket *transmit_cmd);
#endif