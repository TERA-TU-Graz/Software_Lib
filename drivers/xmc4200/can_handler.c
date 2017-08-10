/*
 * can_handler.c
 *
 * ToDo: add file description here
 *
 * Copyright 2017 PÃ¶schl Rene Copyright and related rights are licensed under the Solderpad Hardware License,
 * Version 0.51 (the â€œLicenseâ€�); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://solderpad.org/licenses/SHL-0.51.
 * Unless required by applicable law or agreed to in writing, software,
 * hardware and materials distributed under this License is distributed on an â€œAS ISâ€� BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */
#include <can_handler.h>
#include <config.h>
#include <DAVE3.h>

TeraCanChannel *_can_handler_channels[CAN_MAX_NUM_CHANNELS];
int8_t _can_handler_num_channels = 0;

//TeraCanMsg _can_handler_receive_buffer[CAN_MAX_NUM_CHANNELS];		//from STM driver
static TeraCanMsg _can_recieve_buffer;

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
  /*
  channel->pRxMsg = (CanRxMsgTypeDef*)&(_can_handler_receive_buffer[_can_handler_num_channels]);
  */
  _can_handler_channels[_can_handler_num_channels] = channel;
  _can_handler_num_channels++;
  return _can_handler_num_channels-1;
}


//message object 1 is transmitt the gerneal transmit msg objc.
int8_t CANx_sendMessage(TeraCanMsg *txMsg, int8_t can_channel){
  TERA_ASSERT_CAN(can_channel >= 0 && can_channel < CAN_MAX_NUM_CHANNELS);
  status_t appStatus = 0xFF;

  if(can_channel >= _can_handler_num_channels){
    return -1;
  }

  //configure message object 1 for sending
  CAN001_MessageHandleType tempMO;
  if(txMsg->IDE == STANDARDTYPE){
	  tempMO.IDExten = STANDARDTYPE;
	  tempMO.IDEMask = STANDARDTYPE;
	  tempMO.Identifier = txMsg->StdId;
  } else {
	  tempMO.IDExten = EXTENDEDTYPE;
	  tempMO.IDEMask = EXTENDEDTYPE;
	  tempMO.Identifier = txMsg->ExtId;
  }
  tempMO.DataLength = (uint8_t)txMsg->DLC;
  tempMO.MsgObjEN = CAN001_ENABLE;
  tempMO.MsgObjType = TRANSMSGOBJ;
  tempMO.IDMask = 0x1FFFFFFF;

  int i;
	for(i = 0; i<txMsg->DLC; i++){
	  tempMO.data[i] = txMsg->Data[i];
	}

  //TODO: check if message object is not currently waiting for transmission
  appStatus = CAN001_ConfigMsgObj(_can_handler_channels[can_channel], &tempMO, 1U);
  if(appStatus != DAVEApp_SUCCESS){
	  return appStatus;
  }
  appStatus = CAN001_SendDataFrame(_can_handler_channels[can_channel], 1U);
  if(appStatus != DAVEApp_SUCCESS){
	  return appStatus;
  }
  return 0;
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
  if(is_extended_id){
    txMsg.IDE = EXTENDEDTYPE;
    txMsg.ExtId = msg_id;
  }
  else{
    txMsg.IDE = STANDARDTYPE;
    txMsg.StdId = msg_id;
  }
  txMsg.DLC = data_length;
  int i;
  for(i = 0; i<data_length; i++){
    txMsg.Data[i] = data[i];
  }

  return CANx_sendMessage(&txMsg, can_channel);

}


void DAVE_CAN_TxCpltCallback(){
	/* Check transmit pending status in LMO1 */
  if(CAN001_GetMOFlagStatus(&CAN001_Handle0,1,TRANSMIT_PENDING) == CAN_SET)
  {
    /* Clear the flag */
    CAN001_ClearMOFlagStatus(&CAN001_Handle0,1,TRANSMIT_PENDING);
  }
  //ToDo: implement
}
void DAVE_CAN_RxCpltCallback(){
	CAN001_MessageHandleType* rec_msg = NULL;

	//check which message object recieved a valid frame (should be LMO2)
	if(CAN001_GetMOFlagStatus(&CAN001_Handle0, 2, RECEIVE_PENDING) == CAN_SET){
		CAN001_ReadMsgObj(&CAN001_Handle0, &CAN001_MessageHandle0_2, 2);		//read the data from the CAN registers into the MessageObject
		rec_msg = &CAN001_MessageHandle0_2;																	//general pointer to the message object (because i don't know which message object triggered the recieved interrupt)
		CAN001_ClearMOFlagStatus(&CAN001_Handle0, 2, RECEIVE_PENDING);
	}

	if(rec_msg != NULL){
		_can_recieve_buffer.IDE = rec_msg->IDExten;
		_can_recieve_buffer.StdId = rec_msg->Identifier;
		_can_recieve_buffer.ExtId = rec_msg->Identifier;
		_can_recieve_buffer.DLC = rec_msg->DataLength;
		for(int i=0; i<_can_recieve_buffer.DLC; i++){
			_can_recieve_buffer.Data[i] = rec_msg->data[i];
		}
		can_msg_received_callback(&_can_recieve_buffer);
	}
}
void DAVE_CAN_ErrorCallback(){
  //ToDo: implement
}

// Implement if needed. Otherwise just return 0
int8_t CANhandler_configFilterAcceptAllMsg(int8_t can_channel){
  return 0;
}

__attribute__((weak)) void can_msg_received_callback(TeraCanMsg *receive_buffer){
  // This should be implemented within a module.
}
