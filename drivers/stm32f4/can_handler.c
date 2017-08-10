/*
 * can_handler.c
 *
 * ToDo: add file description here
 *
 * Copyright 2017 Pöschl Rene Copyright and related rights are licensed under the Solderpad Hardware License,
 * Version 0.51 (the “License”); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://solderpad.org/licenses/SHL-0.51.
 * Unless required by applicable law or agreed to in writing, software,
 * hardware and materials distributed under this License is distributed on an “AS IS” BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */
#include <can_handler.h>
#include <config.h>

TeraCanChannel *_can_handler_channels[CAN_MAX_NUM_CHANNELS];
int8_t _can_handler_num_channels = 0;

TeraCanMsg _can_handler_receive_buffer[CAN_MAX_NUM_CHANNELS];

/* Add a channel to the can handler.
 *
 * @param {TeraCanChannel*} channel a pointer to a valid hal typedef struct.
 * @return {int8_t} returns index of channel or -1 on error.
 */
int8_t CANhandler_addChannel(TeraCanChannel *channel){
  TERA_ASSERT_CAN(channel != NULL);

  if(_can_handler_num_channels >= CAN_MAX_NUM_CHANNELS){
    return -1;
  }
  channel->pRxMsg = (CanRxMsgTypeDef*)&(_can_handler_receive_buffer[_can_handler_num_channels]);
  _can_handler_channels[_can_handler_num_channels] = channel;
  _can_handler_num_channels++;

  uint8_t FIFONumber = CAN_FIFO0;
  if(__HAL_CAN_MSG_PENDING(channel, CAN_FIFO1) > __HAL_CAN_MSG_PENDING(channel, CAN_FIFO0)){
    FIFONumber = CAN_FIFO1;
  }
  HAL_CAN_Receive_IT(channel, FIFONumber);

  return _can_handler_num_channels-1;
}

int8_t CANx_sendMessage(TeraCanMsg *txMsg, int8_t can_channel){
  TERA_ASSERT_CAN(can_channel >= 0 && can_channel < CAN_MAX_NUM_CHANNELS);

  if(can_channel >= _can_handler_num_channels){
    return -1;
  }
  _can_handler_channels[can_channel]->pTxMsg = (CanTxMsgTypeDef*)txMsg;
  return HAL_CAN_Transmit_IT(_can_handler_channels[can_channel]);
}

int8_t CANx_send(uint32_t msg_id, uint8_t *data, uint8_t data_length, bool is_extended_id, int8_t can_channel){
  TERA_ASSERT_CAN(can_channel >= 0 && can_channel < CAN_MAX_NUM_CHANNELS);
  TERA_ASSERT_CAN(data_length <= 8);
  if(can_channel >= _can_handler_num_channels){
    return -1;
  }
  if(data_length > 8){
    return -1;
  }

  TeraCanMsg txMsg;
  txMsg.RTR = CAN_RTR_DATA;
  if(is_extended_id){
    txMsg.IDE = CAN_ID_EXT;
    txMsg.ExtId = msg_id;
  }
  else{
    txMsg.IDE = CAN_ID_STD;
    txMsg.StdId = msg_id;
  }
  txMsg.DLC = data_length;
  int i;
  for(i = 0; i<data_length; i++){
    txMsg.Data[i] = data[i];
  }

  return CANx_sendMessage(&txMsg, can_channel);

}


void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
  //ToDo: implement
}
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){

  can_msg_received_callback((TeraCanMsg*)hcan->pRxMsg);
  uint8_t FIFONumber = CAN_FIFO0;
  if(__HAL_CAN_MSG_PENDING(hcan, CAN_FIFO1) > __HAL_CAN_MSG_PENDING(hcan, CAN_FIFO0)){
    FIFONumber = CAN_FIFO1;
  }

  HAL_CAN_Receive_IT(hcan, FIFONumber); //ToDo: error handling
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
  //ToDo: implement
}

__attribute__((weak)) void can_msg_received_callback(TeraCanMsg *receive_buffer){
  // This should be implemented within a module.
}

//ToDo: Better Filter Handling. (Maybe write a FilterHandler)
//
int8_t CANhandler_configFilterAcceptAllMsg(int8_t can_channel){
  CAN_FilterConfTypeDef filter_config;

  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterIdHigh = 0x0000;
  filter_config.FilterIdLow = 0x0000;
  filter_config.FilterMaskIdHigh = 0x0000;
  filter_config.FilterMaskIdLow = 0x0000;

  if(_can_handler_channels[can_channel]->Instance == CAN2){
    filter_config.FilterNumber = CAN_CAN2_FIRST_FILTER_BANK_NR;
  }
  else{
    filter_config.FilterNumber = 0;
  }

  filter_config.BankNumber = CAN_CAN2_FIRST_FILTER_BANK_NR;
  filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter_config.FilterActivation = ENABLE;


  return HAL_CAN_ConfigFilter(NULL, &filter_config);
}
