/*
 * can_handler.h
 *
 * ToDo: add file description here
 *
 * Copyright 2017 Pöschl Rene Copyright and related rights are licensed under the Solderpad Hardware License,
 * Version 0.51 (the License); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://solderpad.org/licenses/SHL-0.51.
 * Unless required by applicable law or agreed to in writing, software,
 * hardware and materials distributed under this License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */

#include <stdint.h>
#include <stdbool.h>
#include <config/can_handler_def.h>

#ifndef LIBRARIES_TERALIB_DRIVERS_INCLUDE_CAN_HANDLER_H_
#define LIBRARIES_TERALIB_DRIVERS_INCLUDE_CAN_HANDLER_H_


/**
  * @brief  CAN Rx message structure definition
  */
typedef struct
{
  uint32_t StdId;       /*!< Specifies the standard identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId;       /*!< Specifies the extended identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE;         /*!< Specifies the type of identifier for the message that will be received.
                             This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR;         /*!< Specifies the type of frame for the received message.
                             This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC;         /*!< Specifies the length of the frame that will be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];      /*!< Contains the data to be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FMI;         /*!< Specifies the index of the filter the message stored in the mailbox passes through.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FIFONumber;  /*!< Specifies the receive FIFO number.
                             This parameter can be CAN_FIFO0 or CAN_FIFO1 */

}TeraCanMsg;

int8_t CANhandler_addChannel(TeraCanChannel *channel);
int8_t CANx_sendMessage(TeraCanMsg *txMsg, int8_t can_channel);
int8_t CANx_send(uint32_t msg_id, uint8_t *data, uint8_t data_length, bool is_extended_id, int8_t can_channel);

int8_t CANhandler_configFilterAcceptAllMsg(int8_t can_channel);

void can_msg_received_callback(TeraCanMsg *receive_buffer);


#endif /* LIBRARIES_TERALIB_DRIVERS_INCLUDE_CAN_HANDLER_H_ */
