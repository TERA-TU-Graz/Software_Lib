#include <can/can_fennek.h>
#include <can_handler.h>
#include <stdbool.h>
#include <systick.h>


// private variables
static uint32_t _can_fennek_boardID;      // ID der Platien auf dem das Programm grad läuft
static uint16_t _can_fennek_status_mask;  // needed for calc time between status messages
static int8_t _can_fennek_channel;


// variables sent over can
uint32_t can_fennek_data_matrix_[CAN_FENNEK_BOARD_COUNT][CAN_FENNEK_MSG_COUNT];
// status information
uint32_t can_fennek_status_matrix_[CAN_FENNEK_BOARD_COUNT];
// zeit zwischen zwei statusnachrichten
uint32_t can_fennek_last_valid_status_[CAN_FENNEK_BOARD_COUNT];
uint32_t can_fennek_status_error_cnt_[CAN_FENNEK_BOARD_COUNT];

/*
 * initialize the fennek can module. (The hardware and the driver need to be initialized separately!)
 *
 * @param {int8_t} can_channle The CAN channel number returned from the can_handler.
 * @return {int8_t} returns index of 0 on success or -1 on error. // ToDo better error handling
 */

int8_t CANfennek_init(int8_t can_channel, uint32_t board_id){
  _can_fennek_status_mask = 0;
  _can_fennek_channel = -1;
  _can_fennek_boardID = CAN_FENNEK_BOARD_UNKNOWN;

  if(can_channel < 0){
    return -1;
  }
  _can_fennek_channel = can_channel;

  if(board_id >= CAN_FENNEK_BOARD_COUNT){
    return -1;
  }
  _can_fennek_boardID = board_id;

  CANhandler_configFilterAcceptAllMsg(can_channel);

  return 0;
}

//--------------------------------------------------------------------------------------------------
void can_msg_received_callback(TeraCanMsg *receive_buffer){
  uint8_t board_index, msg_index;
  uint32_t data;
  uint32_t type;
  uint32_t boardID;

  // Todo: check if extid, maybe with abstraction?
  type = (receive_buffer->ExtId >> CAN_FENNEK_MSG_TYPE_POS) & CAN_FENNEK_MSG_TYPE_MASK;
  boardID = receive_buffer->ExtId & CAN_FENNEK_BOARD_MASK;

  board_index = receive_buffer->Data[0];
  msg_index = receive_buffer->Data[1];
  data = receive_buffer->Data[2] | receive_buffer->Data[3] << 8 | receive_buffer->Data[4] << 16 | receive_buffer->Data[5] << 24;

  if(board_index >= CAN_FENNEK_BOARD_COUNT || msg_index >= CAN_FENNEK_MSG_COUNT){
    return; //ToDo: errorhandling
  }
  if (type==CAN_FENNEK_MSG_TYPE_DATA) {
    CAN_FENNEK_VARIABLE(board_index, msg_index) = data;
  } else if (type==CAN_FENNEK_MSG_TYPE_STATUS) {
    CAN_FENNEK_STATUS(board_index) = data;
    can_fennek_last_valid_status_[board_index] = TERA_SysTick_ms();
  }
}

//--------------------------------------------------------------------------------------------------
void CANfennek_status_periodHandler() {
  uint32_t i;
  uint32_t tick_now = TERA_SysTick_ms();
  for(i=0;i<CAN_FENNEK_BOARD_COUNT;++i) {
    if((can_fennek_last_valid_status_[i] - tick_now)/1000 > CAN_FENNEK_STATUS_ERROR_TIME) {
      if(can_fennek_status_error_cnt_[i] < UINT32_MAX){
        ++can_fennek_status_error_cnt_[i];
      }
    }
    else{
      can_fennek_status_error_cnt_[i] = 0;
    }
  }

}


//--------------------------------------------------------------------------------------------------
uint32_t CANfennek_getBoardID() {
  return _can_fennek_boardID;
}

//--------------------------------------------------------------------------------------------------
uint8_t CANfennek_sendDataMessage(uint32_t priority, uint8_t fennek_msgID, uint32_t data) {
  if(fennek_msgID >= CAN_FENNEK_MSG_COUNT){
    return -1;
  }

  uint32_t msg_id = (1 << _can_fennek_boardID)
      | (CAN_FENNEK_MSG_TYPE_DATA << CAN_FENNEK_MSG_TYPE_POS)
      | ((priority & CAN_FENNEK_PRIORITY_MASK) << CAN_FENNEK_PRIORITY_POS);

  uint8_t can_data[8];

  can_data[0] = _can_fennek_boardID;
  can_data[1] = fennek_msgID;
  can_data[2] = data & 0xFF;
  can_data[3] = (data>>8) & 0xFF;
  can_data[4] = (data>>16) & 0xFF;
  can_data[5] = (data>>24) & 0xFF;
  can_data[6] = 0;
  can_data[7] = 0;

  return CANx_send(msg_id, can_data, 6, true, _can_fennek_channel);
}

//--------------------------------------------------------------------------------------------------
uint8_t CANfennek_sendStatusMessage(uint32_t priority, uint32_t status) {
  uint32_t msg_id = (1 << _can_fennek_boardID)
        | (CAN_FENNEK_MSG_TYPE_STATUS << CAN_FENNEK_MSG_TYPE_POS)
        | ((priority & CAN_FENNEK_PRIORITY_MASK) << CAN_FENNEK_PRIORITY_POS);

  uint8_t can_data[8];


  can_data[0] = _can_fennek_boardID;
  can_data[1] = 0;
  can_data[2] = status & 0xFF;
  can_data[3] = (status>>8) & 0xFF;
  can_data[4] = (status>>16) & 0xFF;
  can_data[5] = (status>>24) & 0xFF;
  can_data[6] = 0;
  can_data[7] = 0;

  return CANx_send(msg_id, can_data, 6, true, _can_fennek_channel);
}

//--------------------------------------------------------------------------------------------------
int16_t CANfennek_status_getPeriod(uint32_t boardID) {
  return (can_fennek_last_valid_status_[boardID] - TERA_SysTick_ms())/1000;
}

//--------------------------------------------------------------------------------------------------
uint32_t CANfennek_status_getErrorCounter(uint32_t boardID) {
  return can_fennek_status_error_cnt_[boardID];
}

//--------------------------------------------------------------------------------------------------
void CANfennek_status_resetErrorCounter(uint32_t boardID) {
  can_fennek_status_error_cnt_[boardID] = 0;
}
