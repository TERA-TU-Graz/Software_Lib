#ifndef __MODULES__LEGACY__CAN_H__
#define __MODULES__LEGACY__CAN_H__

#include <stdint.h>
#include <can_handler.h>

/*
--------------------------------------------------------------------------------------------------
baudrate
CAN-clk = APB1 (42MHz)

tq = (can-prescaler+1) * tCLK(APB1-clk)

tBS1 = tq * (TS1[3:0] + 1)
tBS2 = tq * (TS2[2:0] + 1)

TS1 = 0-16
TS2 = 0-7
bitTime = tq + tBS1 + tBS2
bautrate = 1/bitTime


bei Verwendung der CAN-Init-Funktion, defines verwenden, CAN_BS1_(1-16)tq
CAN_BS2_(1-8)tq

--------------------------------------------------------------------------------------------------
  CAN_Potokoll
ID - Struktur
 3bit  reserviert
 6bit  priorität
 4bit  nachrichten-typ
16bit  sender-board


Variablen
byte[0] = zeile, board
byte[1] = spalte, variable
byte[2-5] = variable

heartbeat, status

IMPORTANT
benutzer muss sicherstellen, dass die Funktion  void CAN_status_periodHandler()
alle  CAN_STATUS_PERIOD [us] aufgerufen wird

demo code
// Handler
void SysTick_Handler() {
  CAN_status_periodHandler();
}

  // config systick
  SYSTICK_init(SysTick_CLKSource_HCLK_Div8, CAN_STATUS_PERIOD);
*/

// in us
#define CAN_FENNEK_STATUS_PERIOD       50000 // us  entspricht  50ms
#define CAN_FENNEK_STATUS_ERROR_TIME   1000   // 3s

#define CAN_FENNEK_PRIORITY_MASK      0x3F  // 6 bits
#define CAN_FENNEK_PRIORITY_POS        20

//
#define CAN_FENNEK_PRIORITY_0         0x11
#define CAN_FENNEK_PRIORITY_1         0x12
#define CAN_FENNEK_PRIORITY_2         0x13
#define CAN_FENNEK_PRIORITY_3         0x14
#define CAN_FENNEK_PRIORITY_4         0x15
#define CAN_FENNEK_PRIORITY_5         0x16
#define CAN_FENNEK_PRIORITY_6         0x17
#define CAN_FENNEK_PRIORITY_7         0x18
#define CAN_FENNEK_PRIORITY_8         0x19
#define CAN_FENNEK_PRIORITY_9         0x1A
#define CAN_FENNEK_PRIORITY_10        0x2A
#define CAN_FENNEK_PRIORITY_11        0x2B
#define CAN_FENNEK_PRIORITY_12        0x2C
#define CAN_FENNEK_PRIORITY_13        0x2D
#define CAN_FENNEK_PRIORITY_14        0x2E
#define CAN_FENNEK_PRIORITY_15        0x2F


#define CAN_FENNEK_MSG_TYPE_MASK         0xF
#define CAN_FENNEK_MSG_TYPE_POS          16

#define CAN_FENNEK_MSG_TYPE_STATUS       0x00
#define CAN_FENNEK_MSG_TYPE_DATA         0x01
#define CAN_FENNEK_MSG_TYPE_CALLBACK     0x02
#define CAN_FENNEK_MSG_TYPE_FREE1        0x04
#define CAN_FENNEK_MSG_TYPE_FREE2        0x08

// zeile in der matrix
// vorhandene boards
// maximale anzahl an boards
#define CAN_FENNEK_BOARD_MASK             0xFFFF
#define CAN_FENNEK_BOARD_COUNT                16  // zeilen in der matrix
#define CAN_FENNEK_MSG_COUNT                  32  // anzahl der variablen

#define CAN_FENNEK_BOARD_UNKNOWN              15
#define CAN_FENNEK_MSG_UNKNOWN                31


#define CAN_FENNEK_VARIABLE(board, msg) (can_fennek_data_matrix_[board][msg])
#define CAN_FENNEK_STATUS(board) (can_fennek_status_matrix_[board])

// zugriff [zeile][spalte]
extern uint32_t can_fennek_data_matrix_[CAN_FENNEK_BOARD_COUNT][CAN_FENNEK_MSG_COUNT];
extern uint32_t can_fennek_status_matrix_[CAN_FENNEK_BOARD_COUNT];


//--------------------------------------------------------------------------------------------------
// public functions
void CANfennek_status_periodHandler();
int8_t CANfennek_init(int8_t can_channel, uint32_t board_id);



uint32_t CANfennek_getBoardID();
// send raw message

// send data message
uint8_t CANfennek_sendDataMessage(uint32_t priority, uint8_t msgID, uint32_t data);
// send status message
uint8_t CANfennek_sendStatusMessage(uint32_t priority, uint32_t status);

int16_t CANfennek_status_getPeriod(uint32_t boardID);
uint32_t CANfennek_status_getErrorCounter(uint32_t boardID);
void CANfennek_status_resetErrorCounter(uint32_t boardID);

#endif // __MODULES__LEGACY__CAN_H__
