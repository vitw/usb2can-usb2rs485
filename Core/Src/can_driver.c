#include "main.h"
#include "cmsis_os.h"
#include "board.h"
#include "can_driver.h"
#include "can.h"
#include "usbd_cdc_if.h"

extern CAN_HandleTypeDef hcan2;
extern uint8_t UserTxBufferFS[];

#define CONVERT_SJW_to_bxCAN_SJW(__SJW__) ((((__SJW__)-1) << 24) & (0x03 << 24))
#define CONVERT_BS1_to_bxCAN_BS1(__BS1__) ((((__BS1__)-1) << 16) & (0x0F << 16))
#define CONVERT_BS2_to_bxCAN_BS2(__BS2__) ((((__BS2__)-1) << 20) & (0x07 << 20))
#define CAN_NUMB_SPEED 9

typedef struct {
  uint32_t Speed;
  uint8_t SJW;
  uint8_t BS1;
  uint8_t BS2;
  uint32_t Prescaler;
} tCANTimingParams;

const tCANTimingParams CANTimingParams[CAN_NUMB_SPEED] = {
  { 10000, 1, 13, 2, 225 }, // Sample point at 87.5%
  { 20000, 1, 15, 2, 100 }, // Sample point at 88.9%
  { 50000, 1, 13, 2, 45 }, // Sample point at 87.5%
  { 100000, 1, 15, 2, 20 }, // Sample point at 88.9%
  { 125000, 1, 13, 2, 18 }, // Sample point at 87.5%
  { 250000, 1, 13, 2, 9 }, // Sample point at 87.5%
  { 500000, 1, 15, 2, 4 }, // Sample point at 88.9%
  { 800000, 1, 7, 1, 5 }, // Sample point at 88.9%
  //{  800000, 1,  10, 1,   3 },	// Sample point at 91.7%
  { 1000000, 1, 15, 2, 2 } // Sample point at 88.9%
};

uint8_t CANSetSpeed(uint8_t speedcod)
{
  uint8_t resfuc = 0;
  speedcod -= '0';
  hcan2.Init.SyncJumpWidth = CONVERT_SJW_to_bxCAN_SJW(CANTimingParams[speedcod].SJW);
  hcan2.Init.TimeSeg1 = CONVERT_BS1_to_bxCAN_BS1(CANTimingParams[speedcod].BS1);
  hcan2.Init.TimeSeg2 = CONVERT_BS2_to_bxCAN_BS2(CANTimingParams[speedcod].BS2);
  hcan2.Init.Prescaler = CANTimingParams[speedcod].Prescaler;
  HAL_CAN_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  return 0;
}

uint8_t can_transmit(dataPacket* transmit_cmd)
{
  slcanCmd cmd;
  uint32_t num_bytes = 0;
  uint8_t all_data_len = transmit_cmd->data_len;
  uint8_t cur_data_index = 0;
  while (all_data_len > 0) {
    switch (transmit_cmd->data[cur_data_index]) {
    case 'S':
      cmd.cmd_type = SET_BIT_RATE;
      CANSetSpeed(transmit_cmd->data[cur_data_index + 1]);
      all_data_len -= 3;
      cur_data_index += 3;
      break;
    case 'O':
      cmd.cmd_type = OPEN_CHANNEL;
      num_bytes = sprintf((char*)UserTxBufferFS, "\r");
      all_data_len -= 2;
      cur_data_index += 2;
      break;
    case 'C':
      cmd.cmd_type = CLOSE_CHANNEL;
      num_bytes = sprintf((char*)UserTxBufferFS, "\r");
      all_data_len -= 2;
      cur_data_index += 2;
      break;
    case 't': {
      uint32_t can_id = 0;
      uint32_t mb;
      uint8_t data_len = 0;
      CAN_TxHeaderTypeDef can_msg;
      cmd.cmd_type = TRANSMIT_STANDART_ID;
      sscanf(transmit_cmd->data, "t%03x%01d", &can_id, &data_len);
      if (data_len <= 8) {
          memcpy((void*)cmd.value, (void*)&transmit_cmd->data[1], ((data_len * 2) + 4));
      }
      can_msg.StdId = can_id;
      can_msg.ExtId = 0;
      can_msg.DLC = data_len;
      can_msg.IDE = CAN_ID_STD;
      can_msg.RTR = CAN_RTR_DATA;
      can_msg.TransmitGlobalTime = DISABLE;
      HAL_CAN_AddTxMessage(&hcan2, &can_msg, &transmit_cmd->data[5], &mb);
      num_bytes = sprintf((char*)UserTxBufferFS, "z\r");
      all_data_len = 0;
      break;
    }
    case 'V':
      cmd.cmd_type = GET_VERSION;
      num_bytes = sprintf((char*)UserTxBufferFS, "V0101\r");
      all_data_len -= 2;
      cur_data_index += 2;
      break;
    default:
      cmd.cmd_type = END_CMD;
      num_bytes = sprintf((char*)UserTxBufferFS, "\r");
      all_data_len = 0;
      break;
    }
  }
  CDC_Transmit_FS(UserTxBufferFS, num_bytes);
  return 0;
}
