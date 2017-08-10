#ifndef TERA_LIBRARY_CAN_H_INCLUDED
#define TERA_LIBRARY_CAN_H_INCLUDED

#if defined(PROJECT_FENNEK) || defined(PROJECT_PANTHER)
#include "module/common/modules_def.h"

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
#define CAN_STATUS_PERIOD       50000 // us  entspricht  50ms
#define CAN_STATUS_ERROR_TIME   1000   // 3s

#define CAN_PRIORITY_MASK      0x3F  // 6 bits
#define CAN_PRIORITY_POS        20

//
#define CAN_PRIORITY_0         0x11
#define CAN_PRIORITY_1         0x12
#define CAN_PRIORITY_2         0x13
#define CAN_PRIORITY_3         0x14
#define CAN_PRIORITY_4         0x15
#define CAN_PRIORITY_5         0x16
#define CAN_PRIORITY_6         0x17
#define CAN_PRIORITY_7         0x18
#define CAN_PRIORITY_8         0x19
#define CAN_PRIORITY_9         0x1A
#define CAN_PRIORITY_10        0x2A
#define CAN_PRIORITY_11        0x2B
#define CAN_PRIORITY_12        0x2C
#define CAN_PRIORITY_13        0x2D
#define CAN_PRIORITY_14        0x2E
#define CAN_PRIORITY_15        0x2F


#define CAN_MSG_TYPE_MASK         0xF
#define CAN_MSG_TYPE_POS          16

#define CAN_MSG_TYPE_STATUS       0x00
#define CAN_MSG_TYPE_DATA         0x01
#define CAN_MSG_TYPE_CALLBACK     0x02
#define CAN_MSG_TYPE_FREE1        0x04
#define CAN_MSG_TYPE_FREE2        0x08

// zeile in der matrix
// vorhandene boards
// maximale anzahl an boards
#define CAN_BOARD_MASK             0xFFFF
#define CAN_BOARD_COUNT                16  // zeilen in der matrix
#define CAN_MSG_COUNT                  32  // anzahl der variablen

#define CAN_BOARD_UNKNOWN              15
#define CAN_MSG_UNKNOWN                31


#define CAN_VARIABLE(board, msg) (can_data_matrix_[board][msg])
#define CAN_STATUS(board) (can_status_matrix_[board])

// zugriff [zeile][spalte]
extern uint32_t can_data_matrix_[CAN_BOARD_COUNT][CAN_MSG_COUNT];
extern uint32_t can_status_matrix_[CAN_BOARD_COUNT];



//--------------------------------------------------------------------------------------------------
// port pin definition
#if CONFIG_UC_BOARD==UC_BOARD_STM3240G_EVAL

#define CAN_USE_CAN2

#define CANx                       CAN2
#define CAN_IRQ_CHANNEL            CAN2_RX0_IRQn
#define CAN_CLK                    (RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2)
#define CAN_AF                     GPIO_AF_CAN2

#define CAN_RX_PIN                 GPIO_Pin_5
#define CAN_RX_GPIO_PORT           GPIOB
#define CAN_RX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CAN_RX_PIN_AF              GPIO_AF_CAN2
#define CAN_RX_SOURCE              GPIO_PinSource5

#define CAN_TX_PIN                 GPIO_Pin_13
#define CAN_TX_GPIO_PORT           GPIOB
#define CAN_TX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CAN_TX_PIN_AF              GPIO_AF_CAN2
#define CAN_TX_SOURCE              GPIO_PinSource13

#elif CONFIG_UC_BOARD==UC_BOARD_STM32F4TERA

#define CAN_USE_CAN1

#define CANx                       CAN1
#define CAN_IRQ_CHANNEL            CAN1_RX0_IRQn
#define CAN_CLK                    RCC_APB1Periph_CAN1
#define CAN_AF                     GPIO_AF_CAN1

#define CAN_RX_PIN                 GPIO_Pin_8
#define CAN_RX_GPIO_PORT           GPIOB
#define CAN_RX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CAN_RX_PIN_AF              GPIO_AF_CAN1
#define CAN_RX_SOURCE              GPIO_PinSource8

#define CAN_TX_PIN                 GPIO_Pin_9
#define CAN_TX_GPIO_PORT           GPIOB
#define CAN_TX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CAN_TX_PIN_AF              GPIO_AF_CAN1
#define CAN_TX_SOURCE              GPIO_PinSource9

#elif CONFIG_UC_BOARD==UC_BOARD_STM32F4TERA_V2

#define CAN_USE_CAN1

#define CANx                       CAN1
#define CAN_IRQ_CHANNEL            CAN1_RX0_IRQn
#define CAN_CLK                    RCC_APB1Periph_CAN1
#define CAN_AF                     GPIO_AF_CAN1

#define CAN_RX_PIN                 GPIO_Pin_11
#define CAN_RX_GPIO_PORT           GPIOA
#define CAN_RX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define CAN_RX_PIN_AF              GPIO_AF_CAN1
#define CAN_RX_SOURCE              GPIO_PinSource11

#define CAN_TX_PIN                 GPIO_Pin_12
#define CAN_TX_GPIO_PORT           GPIOA
#define CAN_TX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define CAN_TX_PIN_AF              GPIO_AF_CAN1
#define CAN_TX_SOURCE              GPIO_PinSource12

#endif




typedef struct {
  CAN_TypeDef *can;
  uint32_t clock;

  GPIO2 pins[2];
  uint8_t af;

  IRQn_Type irq;
} CAN_Config;


//--------------------------------------------------------------------------------------------------
// public functions

void CANx_RX0_IRQHandler();
void CAN_status_periodHandler();

#if CONFIG_UC_BOARD==UC_BOARD_CODE_SELECT
void CAN_init(CAN_Config *config, uint32_t boardID);
#else
void CAN_init(uint32_t boardID);
#endif


uint32_t CAN_getBoardID();
// send raw message
uint8_t CAN_sendMessage(CanTxMsg *txMsg);
// send data message
uint8_t CAN_sendDataMessage(uint32_t priority, uint32_t msgID, uint32_t data);
// send status message
uint8_t CAN_sendStatusMessage(uint32_t priority, uint32_t status);

int16_t CAN_status_getPeriod(uint32_t boardID);
uint32_t CAN_status_getErrorCounter(uint32_t boardID);
void CAN_status_resetErrorCounter(uint32_t boardID);

#endif

#if defined(PROJECT_IBEX)

#include "can_defines.h"
#include <module/common/modules_def.h>

#if !defined(MODULE_CAN_TIMER) || !defined(MODULE_CAN_TIMER_IRQHANDLER)
// use a default timer... compiler cries anyway if handler is multiply defined
#define MODULE_CAN_TIMER TIM6_CH1
#define MODULE_CAN_TIMER_IRQHANDLER TIM6_DAC_IRQHandler
#endif

enum CAN_eBaudrate { BAUD_1M = 2, BAUD_500K = 4, BAUD_250K = 8, BAUD_125K = 16};

//--------------------------------------------------------------------------------------------------
// This function lets you declare 2 filters. Short explanation:
// With id1/2 you specify a bitmask. Witch match_mask1/2 you specify which of the bits in id1/2
// *must* match and which are "don't care".
// Example:                id = 10101101
//                 match_mask = 01110011
//          received_messages = x010xx01 ,where x can be arbitrary
//
// @param CAN_TypeDef* canx     Specify if filter applies to CAN1 or CAN2
// @param uint16_t id1          Word to apply the matching mask (match_mask1) on
// @param uint16_t match_mask1  Bitmask which specifies which bits in id1 *must* match (where mask bits=1) and which are arbitrary (where mask bits=0)
// @param uint16_t id2          Word to apply the matching mask (match_mask2) on
// @param uint16_t match_mask2  Bitmask which specifies which bits in id2 *must* match (where mask bits=1) and which are arbitrary (where mask bits=0)
//
// @return 0 on success, -1 if no more free filter banks
//
int CAN_addMaskFilters16Bit(CAN_TypeDef* canx, uint16_t id1, uint16_t match_mask1, uint16_t id2, uint16_t match_mask2);

//--------------------------------------------------------------------------------------------------
// Specifies 4 ID filters. Only message ids which exactly match one of the filters are not dropped.
//
// @param CAN_TypeDef* canx     Specify if filter applies to CAN1 or CAN2
// @param uint16_t id1          First filter identifier
// @param uint16_t id2          Second filter identifier
// @param uint16_t id3          Third filter identifier
// @param uint16_t id4          Fourth filter identifier
//
// @return 0 on success, -1 if no more free filter banks
//
int CAN_addIdFilter16Bit(CAN_TypeDef* CANX, uint16_t id1, uint16_t id2, uint16_t id3, uint16_t id4);

//--------------------------------------------------------------------------------------------------
// Initializes the CAN with the given peripherals
//
// @param enum CAN_eBoardId board_id    Id of this board which is sent in all further communication
// @param CANRx_MyDef* can_rx           The CAN_RX peripheral to use
// @param CANTx_MyDef* can_tx           The CAN_TX peripheral to use
// @param uint8_t irq_priority          Interrupt priority of the CANx_RX and CANx_TX interrupts
// @param uint8_t irq_subpriority       Interrupt subpriority of the CANx_RX and CANx_TX interrupts
// @param enum CAN_eBaudrate baudrate   Baudrate to use on this CAN
//
// @return 0 on success, -1 on failure
//
int CAN_init(enum CAN_eBoardId board_id, CAN_RxPin* can_rx, CAN_TxPin* can_tx,
             uint8_t irq_priority, uint8_t irq_subpriority, enum CAN_eBaudrate baudrate);

//--------------------------------------------------------------------------------------------------
// Write data on specified CAN.
//
// @param CAN_TypeDef canx              The CAN peripheral to werite to. Either CAN1 or CAN2
// @param enum CAN_eMessageId msg_id    The id of the message to be sent
// @param uint8_t* data                 Pointer to the data bytes which will be sent
// @param uint8_t length_in_bytes       Number of data bytes to write
//
// @return 0 on success, -1 if no free mailbox
//
int CAN_writeData(CAN_TypeDef* canx, enum CAN_eMessageId msg_id, uint8_t* data, uint8_t length_in_bytes);

//--------------------------------------------------------------------------------------------------
// Write data on specified CAN. When data is not transmitted before timeout is reached, the
// timeout function will be called.
//
// @param CAN_TypeDef canx              The CAN peripheral to werite to. Either CAN1 or CAN2
// @param enum CAN_eMessageId msg_id    The id of the message to be sent
// @param uint8_t* data                 Pointer to the data bytes which will be sent
// @param uint8_t length_in_bytes       Number of data bytes to write
// @param uint8t_t timeout              Timeout in multiples of timeout timer period
// @param void (*timeout_function)()    The function to call if the timeout occurs
//
// @return 0 on success, -1 if no free mailbox
//
int CAN_writeDataTimeout(CAN_TypeDef* canx, enum CAN_eMessageId msg_id, uint8_t* data, uint8_t length_in_bytes, uint8_t timeout, void (*timeout_function)());

//--------------------------------------------------------------------------------------------------
// Write Remote Transmission Request on specified CAN.
//
// @param CAN_TypeDef canx              The CAN peripheral to werite to. Either CAN1 or CAN2
// @param enum CAN_eMessageId msg_id    The id of the message to be sent
//
// @return 0 on success, -1 if no free mailbox
//
int CAN_writeRemoteTransmissionRequest(CAN_TypeDef* canx, enum CAN_eMessageId msg_id);

//--------------------------------------------------------------------------------------------------
// Write Remote Transmission Request on specified CAN. When request is not transmitted before
// timeout is reached, the timeout function will be called.
// @param CAN_TypeDef canx              The CAN peripheral to werite to. Either CAN1 or CAN2
// @param enum CAN_eMessageId msg_id    The id of the message to be sent
// @param uint8t_t timeout              Timeout in multiples of timeout timer period
// @param void (*timeout_function)()    The function to call if the timeout occurs
// @return 0 on success, -1 if no free mailbox
//
int CAN_writeRemoteTransmissionRequestTimeout(CAN_TypeDef* canx, enum CAN_eMessageId msg_id, uint8_t timeout, void (*timeout_function)());

//--------------------------------------------------------------------------------------------------
// Read message from specified CAN.
//
// @param CAN_TypeDef* canx               read from CAN1 or CAN2
// @param enum CAN_eMessageId* msg_id     Storage address of message ID
// @param enum CAN_eBoardId* board_id     Storage address of board ID
// @param uint8_t* data                   Storage address of data. Attention: up to 8 bytes!
// @param uint8_t* data_length_in_bytes   Storage address of data length code (DLC)
// @return always 0
//
int CAN_read(CAN_TypeDef* canx, enum CAN_eMessageId* msg_id, enum CAN_eBoardId* board_id, uint8_t* data, uint8_t* data_length_in_bytes);

#else
  #error "unknown project. Either compile with make USER_DEFINES=\"PROJECT_IBEX\" or put define in config file"
#endif

#endif // TERA_LIBRARY_CAN_H_INCLUDED
