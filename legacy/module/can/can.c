#if defined(PROJECT_FENNEK) || defined(PROJECT_PANTHER)

#include "can_legacy.h"

#include <stdint.h>

#include <misc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

#if CONFIG_UC_BOARD==UC_BOARD_CODE_SELECT
static CAN_TypeDef* can;
#endif

// private variables
static uint32_t can_boardID_;// ID der Platien auf dem das Programm grad läuft
static uint16_t can_status_mask_;// needed for calc time between status messages
static uint32_t can_status_period_tmp_[CAN_BOARD_COUNT];

// variables sent over can
uint32_t can_data_matrix_[CAN_BOARD_COUNT][CAN_MSG_COUNT];
// status information
uint32_t can_status_matrix_[CAN_BOARD_COUNT];
// zeit zwischen zwei statusnachrichten
uint16_t can_status_period_[CAN_BOARD_COUNT];
uint32_t can_status_error_cnt_[CAN_BOARD_COUNT];

#if CONFIG_UC_BOARD!=UC_BOARD_CODE_SELECT
#ifdef CAN_USE_CAN1
void CAN1_RX0_IRQHandler()
#elif defined(CAN_USE_CAN2)
void CAN2_RX0_IRQHandler()
#endif
{
    CANx_RX0_IRQHandler();
}
#endif

//--------------------------------------------------------------------------------------------------
void CANx_RX0_IRQHandler() {
    CanRxMsg rxMsg;
    uint8_t board_index, msg_index;
    uint32_t data;
    uint32_t type;
    uint32_t boardID;

#if CONFIG_UC_BOARD==UC_BOARD_CODE_SELECT
    CAN_Receive(can, CAN_FIFO0, &rxMsg);
#else
    CAN_Receive(CANx, CAN_FIFO0, &rxMsg);
#endif

    type = (rxMsg.ExtId >> CAN_MSG_TYPE_POS) & CAN_MSG_TYPE_MASK;
    boardID = rxMsg.ExtId & CAN_BOARD_MASK;

    board_index = rxMsg.Data[0];
    msg_index = rxMsg.Data[1];
    data = rxMsg.Data[2] | rxMsg.Data[3] << 8 | rxMsg.Data[4] << 16 | rxMsg.Data[5] << 24;

    if (type==CAN_MSG_TYPE_DATA) {
        CAN_VARIABLE(board_index, msg_index) = data;
    } else if (type==CAN_MSG_TYPE_STATUS) {
        CAN_STATUS(board_index) = data;
        can_status_mask_ |= boardID;
    }
}

//--------------------------------------------------------------------------------------------------
void CAN_status_periodHandler() {
    uint32_t i;
    uint32_t mask;

    for (i=0;i<CAN_BOARD_COUNT;++i) {
        mask = 0x01 << i;
        if (can_status_mask_ & mask) {
            can_status_period_[i] = can_status_period_tmp_[i];
            can_status_period_tmp_[i] = 0;
            can_status_error_cnt_[i] = 0;
        } else {
            if (can_status_period_tmp_[i]>CAN_STATUS_ERROR_TIME) {
                can_status_period_[i] = -1;
                can_status_period_tmp_[i] = 0;
                if (can_status_error_cnt_[i]<UINT32_MAX)
                ++can_status_error_cnt_[i];
            } else
            can_status_period_tmp_[i] += CAN_STATUS_PERIOD/1000; // convert to ms
        }
    }

    can_status_mask_ = 0;
}

//--------------------------------------------------------------------------------------------------
#if CONFIG_UC_BOARD==UC_BOARD_CODE_SELECT

// code select init
void CAN_init(CAN_Config *config, uint32_t boardID) {
    NVIC_InitTypeDef NVIC_InitStructure;
    CAN_InitTypeDef can_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    uint16_t i, j;

    can = config->can;

    can_boardID_ = (boardID<CAN_BOARD_COUNT) ? boardID : CAN_BOARD_UNKNOWN;

    for (i=0;i<CAN_BOARD_COUNT;++i)
    for (j=0;j<CAN_MSG_COUNT;++j)
    can_data_matrix_[i][j] = 0;

    for (i=0;i<CAN_BOARD_COUNT;++i)
    can_status_matrix_[i] = can_status_period_[i] = can_status_period_tmp_[i] = can_status_error_cnt_[i] = 0;

    can_status_mask_ = 0;

    RCC_APB1PeriphClockCmd(config->clock, ENABLE);

    CAN_DeInit(config->can);

    GPIO_init(config->pins, 2, GPIO_Mode_AF, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    GPIO_AFConfig(config->pins, 2, config->af);

    CAN_StructInit(&can_InitStructure);

    // CAN cell init
    can_InitStructure.CAN_TTCM = DISABLE;
    can_InitStructure.CAN_ABOM = DISABLE;
    can_InitStructure.CAN_AWUM = DISABLE;
    can_InitStructure.CAN_NART = DISABLE;
    can_InitStructure.CAN_RFLM = DISABLE;
    can_InitStructure.CAN_TXFP = DISABLE;
    can_InitStructure.CAN_Mode = CAN_Mode_Normal;
    can_InitStructure.CAN_SJW = CAN_SJW_1tq;
    can_InitStructure.CAN_BS1 = CAN_BS1_12tq;
    can_InitStructure.CAN_BS2 = CAN_BS2_8tq;
    // Baudrare festlegen
    can_InitStructure.CAN_Prescaler = 2;// 1MBit
    //can_InitStructure.CAN_Prescaler =  4; // 500kBit
    //can_InitStructure.CAN_Prescaler =  8; // 250kBit
    //can_InitStructure.CAN_Prescaler = 16; // 125kBit
    CAN_Init(config->can, &can_InitStructure);

    // CAN filter init
    // accept all messages
    if (config->can==CAN1)
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    else
    CAN_FilterInitStructure.CAN_FilterNumber = 14;

    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    VECTOR_setIRQHandler(config->irq, CANx_RX0_IRQHandler);

    NVIC_InitStructure.NVIC_IRQChannel = config->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(config->can, CAN_IT_FMP0, ENABLE);
}

#else

void CAN_init(uint32_t boardID) {
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef gpio_InitStructure;
    CAN_InitTypeDef can_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    uint16_t i, j;

    can_boardID_ = (boardID<CAN_BOARD_COUNT) ? boardID : CAN_BOARD_UNKNOWN;

    for (i=0;i<CAN_BOARD_COUNT;++i)
    for (j=0;j<CAN_MSG_COUNT;++j)
    can_data_matrix_[i][j] = 0;

    for (i=0;i<CAN_BOARD_COUNT;++i)
    can_status_matrix_[i] = can_status_period_[i] = can_status_period_tmp_[i] = can_status_error_cnt_[i] = 0;

    can_status_mask_ = 0;

    RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

    CAN_DeInit(CANx);

    RCC_AHB1PeriphClockCmd(CAN_RX_GPIO_CLK | CAN_RX_GPIO_CLK, ENABLE);

    gpio_InitStructure.GPIO_OType = GPIO_OType_PP;
    gpio_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    gpio_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // Configure CAN pin: RX
    gpio_InitStructure.GPIO_Pin = CAN_RX_PIN;
    GPIO_Init(CAN_RX_GPIO_PORT, &gpio_InitStructure);

    // Configure CAN pin: TX
    gpio_InitStructure.GPIO_Pin = CAN_TX_PIN;
    GPIO_Init(CAN_TX_GPIO_PORT, &gpio_InitStructure);

    GPIO_PinAFConfig(CAN_RX_GPIO_PORT, CAN_RX_SOURCE, CAN_AF);
    GPIO_PinAFConfig(CAN_TX_GPIO_PORT, CAN_TX_SOURCE, CAN_AF);

    CAN_StructInit(&can_InitStructure);

    // CAN cell init
    can_InitStructure.CAN_TTCM = DISABLE;
    can_InitStructure.CAN_ABOM = DISABLE;
    can_InitStructure.CAN_AWUM = DISABLE;
    can_InitStructure.CAN_NART = DISABLE;
    can_InitStructure.CAN_RFLM = DISABLE;
    can_InitStructure.CAN_TXFP = DISABLE;
    can_InitStructure.CAN_Mode = CAN_Mode_Normal;
    can_InitStructure.CAN_SJW = CAN_SJW_1tq;
    can_InitStructure.CAN_BS1 = CAN_BS1_12tq;
    can_InitStructure.CAN_BS2 = CAN_BS2_8tq;
    // Baudrare festlegen
    can_InitStructure.CAN_Prescaler = 2;// 1MBit
//  can_InitStructure.CAN_Prescaler =  4; // 500kBit
//  can_InitStructure.CAN_Prescaler =  8; // 250kBit
    //can_InitStructure.CAN_Prescaler = 16; // 125kBit
    CAN_Init(CANx, &can_InitStructure);

    // CAN filter init
    // accept all messages
#if defined(CAN_USE_CAN2)
    CAN_FilterInitStructure.CAN_FilterNumber = 14;
#else
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
#endif
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN_IRQ_CHANNEL;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}
#endif

//--------------------------------------------------------------------------------------------------
uint32_t CAN_getBoardID() {
    return can_boardID_;
}

//--------------------------------------------------------------------------------------------------
uint8_t CAN_sendMessage(CanTxMsg *txMsg) {
#if CONFIG_UC_BOARD==UC_BOARD_CODE_SELECT
    return CAN_Transmit(can, txMsg);
#else
    return CAN_Transmit(CANx, txMsg);
#endif
}

//--------------------------------------------------------------------------------------------------
uint8_t CAN_sendDataMessage(uint32_t priority, uint32_t msgID, uint32_t data) {
    CanTxMsg txMsg;

    txMsg.IDE = CAN_Id_Extended;
    txMsg.RTR = CAN_RTR_Data;
    txMsg.StdId = 0;

    txMsg.ExtId = (1 << can_boardID_);
    txMsg.ExtId |= (CAN_MSG_TYPE_DATA << CAN_MSG_TYPE_POS);
    priority &= CAN_PRIORITY_MASK;
    txMsg.ExtId |= (priority << CAN_PRIORITY_POS);

    txMsg.DLC = 6;
    txMsg.Data[0] = can_boardID_;
    txMsg.Data[1] = msgID;
    txMsg.Data[2] = data & 0xFF;
    txMsg.Data[3] = (data>>8) & 0xFF;
    txMsg.Data[4] = (data>>16) & 0xFF;
    txMsg.Data[5] = (data>>24) & 0xFF;

#if CONFIG_UC_BOARD==UC_BOARD_CODE_SELECT
    return CAN_Transmit(can, &txMsg);
#else
    return CAN_Transmit(CANx, &txMsg);
#endif
}

//--------------------------------------------------------------------------------------------------
uint8_t CAN_sendStatusMessage(uint32_t priority, uint32_t status) {
    CanTxMsg txMsg;

    txMsg.IDE = CAN_Id_Extended;
    txMsg.RTR = CAN_RTR_Data;
    txMsg.StdId = 0;

    txMsg.ExtId = (1 << can_boardID_);
    txMsg.ExtId |= (CAN_MSG_TYPE_STATUS << CAN_MSG_TYPE_POS);
    priority &= CAN_PRIORITY_MASK;
    txMsg.ExtId |= (priority << CAN_PRIORITY_POS);

    txMsg.DLC = 6;
    txMsg.Data[0] = can_boardID_;
    txMsg.Data[1] = 0;
    txMsg.Data[2] = status & 0xFF;
    txMsg.Data[3] = (status>>8) & 0xFF;
    txMsg.Data[4] = (status>>16) & 0xFF;
    txMsg.Data[5] = (status>>24) & 0xFF;

#if CONFIG_UC_BOARD==UC_BOARD_CODE_SELECT
    return CAN_Transmit(can, &txMsg);
#else
    return CAN_Transmit(CANx, &txMsg);
#endif
}

//--------------------------------------------------------------------------------------------------
int16_t CAN_status_getPeriod(uint32_t boardID) {
    return can_status_period_[boardID];
}

//--------------------------------------------------------------------------------------------------
uint32_t CAN_status_getErrorCounter(uint32_t boardID) {
    return can_status_error_cnt_[boardID];
}

//--------------------------------------------------------------------------------------------------
void CAN_status_resetErrorCounter(uint32_t boardID) {
    can_status_error_cnt_[boardID] = 0;
}

#endif

#if defined(PROJECT_IBEX)

#include "can.h"
#include <module/can/can_defines.h>
#include <module/common/modules_def.h>
#include <module/timer/timer.h>

#include <misc.h>

#include <stdbool.h>
#include <stdint.h>

typedef void (*timeout_function)(void);
static void CAN_dummyTimeoutFunction() {
}

static struct {
    bool can1_initialized;
    bool can2_initialized;

    uint8_t filter_bank_count_can1;
    uint8_t filter_bank_count_can2;

    enum CAN_eBoardId board_id;

    uint8_t timeout_times[2][3]; //timeouts for each mailbox on CAN1 ([0][x]) and CAN2 ([1][x])
    timeout_function timeout_functions[2][3]; //called when corresponding timeout_times reach 0

    CanTxMsg HeartbeatMessage;
    Timer_Periph sCANTimer;
} sCANInfo = { false, false, 0, 0, CAN_BOARD_UNKNOWN,
        { { 0, 0, 0 }, { 0, 0, 0 } }, { { CAN_dummyTimeoutFunction,
                CAN_dummyTimeoutFunction, CAN_dummyTimeoutFunction }, {
                CAN_dummyTimeoutFunction, CAN_dummyTimeoutFunction,
                CAN_dummyTimeoutFunction } }, { CAN_MSG_GLOBAL_HEARTBEAT
                << CAN_PARAM_BOARD_ID_BITS | CAN_BOARD_UNKNOWN, 1, CAN_ID_STD,
                CAN_RTR_DATA, 0, { 1, 2, 3, 4, 5, 6, 7, 8 } },
        MODULE_CAN_TIMER };

//--------------------------------------------------------------------------------------------------
//private functions

static void CAN_sendHeartbeatToInitializedCANs() {
    if (sCANInfo.can1_initialized)
        CAN_Transmit(CAN1, &sCANInfo.HeartbeatMessage);
    if (sCANInfo.can2_initialized)
        CAN_Transmit(CAN2, &sCANInfo.HeartbeatMessage);
}

void MODULE_CAN_TIMER_IRQHANDLER() {
    static unsigned heartbeat_count = 0;
    if (TIM_GetITStatus(sCANInfo.sCANTimer.base_addr, TIM_IT_Update)) {
        TIM_ClearITPendingBit(sCANInfo.sCANTimer.base_addr, TIM_IT_Update);

        heartbeat_count = (heartbeat_count + 1)
                        % (CAN_PARAM_TIMEOUT_FREQUENCY / CAN_PARAM_HEARTBEAT_FREQUENCY);
        if (heartbeat_count == 0)
            CAN_sendHeartbeatToInitializedCANs();

        // Call timeout function if timeout reached
        int i = 0;
        for (; i < 3; i++) {
            if (sCANInfo.timeout_times[0][i] > 0) {
                if (--sCANInfo.timeout_times[0][i] == 0) {
                    sCANInfo.timeout_functions[0][i]();
                }
            }
            if (sCANInfo.timeout_times[1][i] > 0) {
                if (--sCANInfo.timeout_times[1][i] == 0) {
                    sCANInfo.timeout_functions[1][i]();
                }
            }
        }
    } else {
        while (1) {
        }
    }
}

void CAN1_TX_IRQHandler() {
    //reset timeout timers
    uint32_t transmit_status = CAN1->TSR;
    if (transmit_status & CAN_TSR_RQCP0) {
        CAN1->TSR |= CAN_TSR_RQCP0;
        sCANInfo.timeout_times[0][0] = 0;
    } else if (transmit_status & CAN_TSR_RQCP1) {
        CAN1->TSR |= CAN_TSR_RQCP1;
        sCANInfo.timeout_times[0][1] = 0;
    } else if (transmit_status & CAN_TSR_RQCP2) {
        CAN1->TSR |= CAN_TSR_RQCP2;
        sCANInfo.timeout_times[0][2] = 0;
    } else
        while (1) {
        }
}

void CAN2_TX_IRQHandler() {
    //reset timeout
    uint32_t transmit_status = CAN2->TSR;
    if (transmit_status & CAN_TSR_RQCP0) {
        CAN2->TSR |= CAN_TSR_RQCP0;
        sCANInfo.timeout_times[1][0] = 0;
    } else if (transmit_status & CAN_TSR_RQCP1) {
        CAN2->TSR |= CAN_TSR_RQCP1;
        sCANInfo.timeout_times[1][1] = 0;
    } else if (transmit_status & CAN_TSR_RQCP2) {
        CAN2->TSR |= CAN_TSR_RQCP2;
        sCANInfo.timeout_times[1][2] = 0;
    } else
        while (1) {
        }
}

//--------------------------------------------------------------------------------------------------
// public functions

int CAN_addMaskFilters16Bit(CAN_TypeDef* canx, uint16_t id1,
        uint16_t match_mask1, uint16_t id2, uint16_t match_mask2) {
    if (sCANInfo.filter_bank_count_can1 >= CAN_PARAM_MAX_FILTER_BANKS_CAN1
            || sCANInfo.filter_bank_count_can2
            >= CAN_PARAM_MAX_FILTER_BANKS_CAN2)
        return -1;

    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    if (canx == CAN1)
        CAN_FilterInitStructure.CAN_FilterNumber =
                sCANInfo.filter_bank_count_can1++; // filter banks CAN1
    else
        CAN_FilterInitStructure.CAN_FilterNumber =
                CAN_PARAM_MAX_FILTER_BANKS_CAN1
                + sCANInfo.filter_bank_count_can2++; // up to CAN_PARAM_MAX_FILTER_BANKS_CAN2 filter banks may be used for CAN2

    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = id1 << 5; //stupid peripheral library is stupid
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = match_mask1 << 5;
    CAN_FilterInitStructure.CAN_FilterIdLow = id2 << 5;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = match_mask2 << 5;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment =
            canx == CAN1 ? CAN_Filter_FIFO0 : CAN_Filter_FIFO1; //CAN1 -> FIFO 0 ; CAN2 -> FIFO 1
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    return 0;
}

int CAN_addIdFilter16Bit(CAN_TypeDef* canx, uint16_t id1, uint16_t id2,
        uint16_t id3, uint16_t id4) {
    if (sCANInfo.filter_bank_count_can1 >= CAN_PARAM_MAX_FILTER_BANKS_CAN1
            || sCANInfo.filter_bank_count_can2
            >= CAN_PARAM_MAX_FILTER_BANKS_CAN2)
        return -1;

    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    if (canx == CAN1)
        CAN_FilterInitStructure.CAN_FilterNumber =
                sCANInfo.filter_bank_count_can1++; // up to 20 filter banks may be used for CAN1
    else
        CAN_FilterInitStructure.CAN_FilterNumber =
                CAN_PARAM_MAX_FILTER_BANKS_CAN1
                + sCANInfo.filter_bank_count_can2++; // up to 8 filter banks may be used for CAN2

    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = id1 << 5; //stupid peripheral library is stupid
    CAN_FilterInitStructure.CAN_FilterIdLow = id2 << 5;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = id3 << 5;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = id4 << 5;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment =
            canx == CAN1 ? CAN_Filter_FIFO0 : CAN_Filter_FIFO1; //CAN1 -> FIFO 0 ; CAN2 -> FIFO 1
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    return 0;
}

int CAN_init(enum CAN_eBoardId board_id, CAN_RxPin* rx_pin, CAN_TxPin* tx_pin,
        uint8_t irq_priority, uint8_t irq_subpriority,
        enum CAN_eBaudrate baudrate) {

    if (tx_pin->can.addr != rx_pin->can.addr)
        return -1;

    // //GPIO configuration

    // Enable GPIO clock
    rx_pin->gpio_pin.rcc_clock_cmd(rx_pin->gpio_pin.rcc_clock, ENABLE);
    tx_pin->gpio_pin.rcc_clock_cmd(tx_pin->gpio_pin.rcc_clock, ENABLE);

    // Connect CAN pins to AF
    GPIO_PinAFConfig(rx_pin->gpio_pin.gpio, rx_pin->gpio_pin.gpio_pinsource,
            rx_pin->gpio_af_can);
    GPIO_PinAFConfig(tx_pin->gpio_pin.gpio, tx_pin->gpio_pin.gpio_pinsource,
            tx_pin->gpio_af_can);

    // Configure CAN RX and TX pins
    GPIO_InitStruct gpio_init;
    gpio_init.mode = GPIO_MODE_AF_PP;
    gpio_init.pull_mode = GPIO_PULL_UP; // TODO CAN - check if this makes sense
    gpio_init.speed = GPIO_SPEED_MEDIUM;

    GPIO_init(&rx_pin->gpio_pin, &gpio_init);
    GPIO_init(&tx_pin->gpio_pin, &gpio_init);

    // // CAN configuration
    // Enable CAN clock
    if (rx_pin->can.addr == CAN2) // CAN2 is slave, needs CAN1 clock enabled
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    rx_pin->can.rcc_clock_cmd(rx_pin->can.rcc_clock, ENABLE); //==can_tx->rcc...

    // CAN register deinit
    CAN_DeInit(rx_pin->can.addr);

    // CAN cell init
    CAN_InitTypeDef CAN_InitStructure;
    CAN_InitStructure.CAN_ABOM = ENABLE;  // auto recover from bus-off error
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE; // enable automatic retransmission
    CAN_InitStructure.CAN_TTCM = DISABLE; // no TTCAN implementation
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE; // transmit higher priority messages first
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

    // Important: Baudrate = CAN_CLK / (CAN_Prescaler * (CAN_SJW + CAN_BS1 + CAN_BS2))!!!!
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
    // (15+1)/(15+5+1) = Sampling point at about 75% (76,19%) allows for more oscillator tolerance
    CAN_InitStructure.CAN_Prescaler = baudrate;
    CAN_Init(rx_pin->can.addr, &CAN_InitStructure);

    sCANInfo.board_id = board_id;

    //initialize heartbeat message
    sCANInfo.HeartbeatMessage.StdId = (CAN_MSG_GLOBAL_HEARTBEAT
            << CAN_PARAM_BOARD_ID_BITS) | board_id;
    sCANInfo.HeartbeatMessage.ExtId = 0x01;
    sCANInfo.HeartbeatMessage.RTR = CAN_RTR_DATA;
    sCANInfo.HeartbeatMessage.IDE = CAN_ID_STD;
    sCANInfo.HeartbeatMessage.DLC = 0;

    // //Interrupt initialization

    NVIC_InitTypeDef NVIC_InitStructure;
    if (rx_pin->can.addr == CAN1)
        NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    else
        NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = irq_priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = irq_subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    if (rx_pin->can.addr == CAN1)
        NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    else
        NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    if (rx_pin->can.addr == CAN1) {
        sCANInfo.can1_initialized = true;

        //FIFO 0 Message pending interrupt
        CAN_ITConfig(rx_pin->can.addr, CAN_IT_FMP0, ENABLE);
    } else {
        sCANInfo.can2_initialized = true;

        //FIFO 1 Message pending interrupt
        CAN_ITConfig(rx_pin->can.addr, CAN_IT_FMP1, ENABLE);
    }

    CAN_ITConfig(rx_pin->can.addr, CAN_IT_TME, ENABLE); //Transmit interrupt

    //initialize CAN timeout/heartbeat timer
    TIMER_initPeriodicInterrupt(&sCANInfo.sCANTimer,
            CAN_PARAM_TIMEOUT_FREQUENCY, CAN_PARAM_TIMEOUT_IRQ_PRIORITY,
            CAN_PARAM_TIMEOUT_IRQ_SUBPRIORITY);

    return 0;
}

int CAN_writeData(CAN_TypeDef* canx, enum CAN_eMessageId msg_id, uint8_t* data,
        uint8_t length_in_bytes) {
    CanTxMsg TxMessage;
    TxMessage.StdId = (msg_id << CAN_PARAM_BOARD_ID_BITS) | sCANInfo.board_id;
    TxMessage.ExtId = 0x01;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = length_in_bytes;

    int i = 0;
    for (; i < length_in_bytes; i++)
        TxMessage.Data[i] = *(data + i);

    uint8_t transmit_mailbox = CAN_Transmit(canx, &TxMessage);

    if (transmit_mailbox == CAN_TxStatus_NoMailBox)
        return -1;

    return 0;
}

int CAN_writeDataTimeout(CAN_TypeDef* canx, enum CAN_eMessageId msg_id,
        uint8_t* data, uint8_t length_in_bytes, uint8_t timeout,
        void (*timeout_function)()) {
    CanTxMsg TxMessage;
    TxMessage.StdId = (msg_id << CAN_PARAM_BOARD_ID_BITS) | sCANInfo.board_id;
    TxMessage.ExtId = 0x01;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = length_in_bytes;

    int i = 0;
    for (; i < length_in_bytes; i++)
        TxMessage.Data[i] = *(data + i);

    uint8_t transmit_mailbox = CAN_Transmit(canx, &TxMessage);

    if (transmit_mailbox == CAN_TxStatus_NoMailBox)
        return -1;

    int storage = canx == CAN1 ? 0 : 1;
    // Timeout can get decremented immediately after setting it (before intended
    // timeout), therefore add 1 time quantum
    sCANInfo.timeout_times[storage][transmit_mailbox] = timeout + 1;
    sCANInfo.timeout_functions[storage][transmit_mailbox] = timeout_function;

    return 0;
}

int CAN_writeRemoteTransmissionRequest(CAN_TypeDef* canx,
        enum CAN_eMessageId msg_id) {
    CanTxMsg TxMessage;
    TxMessage.StdId = (msg_id << CAN_PARAM_BOARD_ID_BITS) | sCANInfo.board_id;
    TxMessage.ExtId = 0x01;
    TxMessage.RTR = CAN_RTR_REMOTE;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = 0;

    uint8_t transmit_mailbox = CAN_Transmit(canx, &TxMessage);

    if (transmit_mailbox == CAN_TxStatus_NoMailBox)
        return -1;

    return 0;
}

int CAN_writeRemoteTransmissionRequestTimeout(CAN_TypeDef* canx,
        enum CAN_eMessageId msg_id, uint8_t timeout, void (*timeout_function)()) {
    CanTxMsg TxMessage;
    TxMessage.StdId = (msg_id << CAN_PARAM_BOARD_ID_BITS) | sCANInfo.board_id;
    TxMessage.ExtId = 0x01;
    TxMessage.RTR = CAN_RTR_REMOTE;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = 0;

    uint8_t transmit_mailbox = CAN_Transmit(canx, &TxMessage);

    if (transmit_mailbox == CAN_TxStatus_NoMailBox)
        return -1;

    int storage = canx == CAN1 ? 0 : 1;
    // Timeout can get decremented immediately after setting it (before intended
    // timeout), therefore add 1 time quantum
    sCANInfo.timeout_times[storage][transmit_mailbox] = timeout + 1;
    sCANInfo.timeout_functions[storage][transmit_mailbox] = timeout_function;

    return 0;
}

int CAN_read(CAN_TypeDef* canx, enum CAN_eMessageId* msg_id,
        enum CAN_eBoardId* board_id, uint8_t* data,
        uint8_t* data_length_in_bytes) {
    CanRxMsg rxMsg;

    if (canx == CAN1)
        CAN_Receive(canx, CAN_FIFO0, &rxMsg);
    else
        CAN_Receive(canx, CAN_FIFO1, &rxMsg);

    *board_id = rxMsg.StdId & CAN_PARAM_BOARD_ID_MASK;
    *msg_id = (rxMsg.StdId & (0x7FF & ~CAN_PARAM_BOARD_ID_MASK))
                    >> CAN_PARAM_BOARD_ID_BITS;

    *data_length_in_bytes = rxMsg.DLC;
    int i = 0;
    for (; i < rxMsg.DLC; i++)
        data[i] = rxMsg.Data[i];

    return 0;
}

#endif //defined(PROJECT_IBEX)
