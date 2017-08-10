#ifndef TERA_LIBRARY_STM32F40_41XXX_DEF_H_INCLUDED
#define TERA_LIBRARY_STM32F40_41XXX_DEF_H_INCLUDED

#ifndef USE_STDPERIPH_DRIVER
#   define USE_STDPERIPH_DRIVER
#endif

#include "../../../drivers/inc/stm32f4/config/stm32f4xx.h"

typedef void (*RCC_ClockCmdFunction)(uint32_t, FunctionalState);

//------------------------------------------------------------------------------
// GPIO
//

typedef GPIO_TypeDef GPIO_Periph;
typedef struct {
    GPIO_Periph* gpio;   /*!< the base address of the GPIO peripheral */
    unsigned     pin_nr; /*!< pin number e.g. 0,1,...,15 */

    uint16_t             gpio_pinsource;
    uint32_t             rcc_clock;
    RCC_ClockCmdFunction rcc_clock_cmd;
} GPIO_Pin;

#define GPIO_PIN_MACRO(port, pin) {              \
    .gpio   = GPIO##port,                        \
    .pin_nr = pin,                    \
    .gpio_pinsource = GPIO_PinSource##pin,       \
    .rcc_clock      = RCC_AHB1Periph_GPIO##port, \
    .rcc_clock_cmd  = RCC_AHB1PeriphClockCmd     \
}

//No PIN
#define GPIO_PIN_NONE  {0,0,0,0,0}

//PA0 - PA15
#define PA0   GPIO_PIN_MACRO(A,0)
#define PA1   GPIO_PIN_MACRO(A,1)
#define PA2   GPIO_PIN_MACRO(A,2)
#define PA3   GPIO_PIN_MACRO(A,3)
#define PA4   GPIO_PIN_MACRO(A,4)
#define PA5   GPIO_PIN_MACRO(A,5)
#define PA6   GPIO_PIN_MACRO(A,6)
#define PA7   GPIO_PIN_MACRO(A,7)
#define PA8   GPIO_PIN_MACRO(A,8)
#define PA9   GPIO_PIN_MACRO(A,9)
#define PA10  GPIO_PIN_MACRO(A,10)
#define PA11  GPIO_PIN_MACRO(A,11)
#define PA12  GPIO_PIN_MACRO(A,12)
#define PA13  GPIO_PIN_MACRO(A,13)
#define PA14  GPIO_PIN_MACRO(A,14)
#define PA15  GPIO_PIN_MACRO(A,15)

//PB0 - PB15
#define PB0   GPIO_PIN_MACRO(B,0)
#define PB1   GPIO_PIN_MACRO(B,1)
#define PB2   GPIO_PIN_MACRO(B,2)
#define PB3   GPIO_PIN_MACRO(B,3)
#define PB4   GPIO_PIN_MACRO(B,4)
#define PB5   GPIO_PIN_MACRO(B,5)
#define PB6   GPIO_PIN_MACRO(B,6)
#define PB7   GPIO_PIN_MACRO(B,7)
#define PB8   GPIO_PIN_MACRO(B,8)
#define PB9   GPIO_PIN_MACRO(B,9)
#define PB10  GPIO_PIN_MACRO(B,10)
#define PB11  GPIO_PIN_MACRO(B,11)
#define PB12  GPIO_PIN_MACRO(B,12)
#define PB13  GPIO_PIN_MACRO(B,13)
#define PB14  GPIO_PIN_MACRO(B,14)
#define PB15  GPIO_PIN_MACRO(B,15)

//PC0 - PC15
#define PC0   GPIO_PIN_MACRO(C,0)
#define PC1   GPIO_PIN_MACRO(C,1)
#define PC2   GPIO_PIN_MACRO(C,2)
#define PC3   GPIO_PIN_MACRO(C,3)
#define PC4   GPIO_PIN_MACRO(C,4)
#define PC5   GPIO_PIN_MACRO(C,5)
#define PC6   GPIO_PIN_MACRO(C,6)
#define PC7   GPIO_PIN_MACRO(C,7)
#define PC8   GPIO_PIN_MACRO(C,8)
#define PC9   GPIO_PIN_MACRO(C,9)
#define PC10  GPIO_PIN_MACRO(C,10)
#define PC11  GPIO_PIN_MACRO(C,11)
#define PC12  GPIO_PIN_MACRO(C,12)
#define PC13  GPIO_PIN_MACRO(C,13)
#define PC14  GPIO_PIN_MACRO(C,14)
#define PC15  GPIO_PIN_MACRO(C,15)

//PD0 - PD15
#define PD0   GPIO_PIN_MACRO(D,0)
#define PD1   GPIO_PIN_MACRO(D,1)
#define PD2   GPIO_PIN_MACRO(D,2)
#define PD3   GPIO_PIN_MACRO(D,3)
#define PD4   GPIO_PIN_MACRO(D,4)
#define PD5   GPIO_PIN_MACRO(D,5)
#define PD6   GPIO_PIN_MACRO(D,6)
#define PD7   GPIO_PIN_MACRO(D,7)
#define PD8   GPIO_PIN_MACRO(D,8)
#define PD9   GPIO_PIN_MACRO(D,9)
#define PD10  GPIO_PIN_MACRO(D,10)
#define PD11  GPIO_PIN_MACRO(D,11)
#define PD12  GPIO_PIN_MACRO(D,12)
#define PD13  GPIO_PIN_MACRO(D,13)
#define PD14  GPIO_PIN_MACRO(D,14)
#define PD15  GPIO_PIN_MACRO(D,15)

//------------------------------------------------------------------------------
// Timer
//

typedef struct {
    TIM_TypeDef* base_addr;

    uint32_t rcc_clock;
    RCC_ClockCmdFunction rcc_clock_cmd;

    uint16_t channel;
    IRQn_Type irq_nr;
} Timer_Periph;

typedef struct {
    GPIO_Pin gpio_pin;
    uint8_t gpio_af_tim;
    Timer_Periph timer;
} TimerPin;

#define TIMER_PERIPH_MACRO(TimerNr, Rcc, Channel, Irqn) {   \
    .base_addr     = TIM##TimerNr,                          \
    .rcc_clock     = Rcc##Periph_TIM##TimerNr,              \
    .rcc_clock_cmd = Rcc##PeriphClockCmd,                   \
    .channel       = Channel,                               \
    .irq_nr        = TIM##TimerNr##_##Irqn                  \
}

#define TIMER_PIN_MACRO(TimerNr, Pin, Rcc, Channel, Irqn) {             \
    .gpio_pin    = Pin,                                                 \
    .gpio_af_tim = GPIO_AF_TIM##TimerNr,                                \
    .timer       = TIMER_PERIPH_MACRO( TimerNr , Rcc, Channel, Irqn),   \
}

//Timer1, irqn can be a value of :
//TIM1_BRK_TIM9_IRQn       TIM1 Break interrupt and TIM9 global interrupt
//TIM1_UP_TIM10_IRQn       TIM1 Update Interrupt and TIM10 global interrupt
//TIM1_TRG_COM_TIM11_IRQn  TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
//TIM1_CC_IRQn             TIM1 Capture Compare Interrupt
#define TIM1_CH2_PA9(Irqn)  TIMER_PIN_MACRO(1, PA9, RCC_APB2, TIM_Channel_2, Irqn)

// Timer2
#define TIM2_CH1_PA0  TIMER_PIN_MACRO(2,  PA0, RCC_APB1, TIM_Channel_1, IRQn)
#define TIM2_CH1_PA5  TIMER_PIN_MACRO(2,  PA5, RCC_APB1, TIM_Channel_1, IRQn)
#define TIM2_CH1_PA15 TIMER_PIN_MACRO(2, PA15, RCC_APB1, TIM_Channel_1, IRQn)
#define TIM2_CH2_PA1  TIMER_PIN_MACRO(2,  PA1, RCC_APB1, TIM_Channel_2, IRQn)

#define TIM2_CH4_PB11 TIMER_PIN_MACRO(2, PB11, RCC_APB1, TIM_Channel_4, IRQn)

// Timer3
#define TIM3_CH1_PA6  TIMER_PIN_MACRO(3,  PA6, RCC_APB1, TIM_Channel_1, IRQn)
#define TIM3_CH1_PB4  TIMER_PIN_MACRO(3,  PB4, RCC_APB1, TIM_Channel_1, IRQn)
#define TIM3_CH1_PC6  TIMER_PIN_MACRO(3,  PC6, RCC_APB1, TIM_Channel_1, IRQn)
#define TIM3_CH2_PA7  TIMER_PIN_MACRO(3,  PA7, RCC_APB1, TIM_Channel_2, IRQn)
#define TIM3_CH2_PB5  TIMER_PIN_MACRO(3,  PB5, RCC_APB1, TIM_Channel_2, IRQn)
#define TIM3_CH2_PC7  TIMER_PIN_MACRO(3,  PC7, RCC_APB1, TIM_Channel_2, IRQn)

#define TIM3_CH4_PC9  TIMER_PIN_MACRO(3,  PC9, RCC_APB1, TIM_Channel_4, IRQn)

//Timer4
#define TIM4_CH1_PB6  TIMER_PIN_MACRO(4,  PB6, RCC_APB1, TIM_Channel_1, IRQn)
#define TIM4_CH2_PB7  TIMER_PIN_MACRO(4,  PB7, RCC_APB1, TIM_Channel_2, IRQn)
#define TIM4_CH3_PB8  TIMER_PIN_MACRO(4,  PB8, RCC_APB1, TIM_Channel_3, IRQn)
#define TIM4_CH4_PB9  TIMER_PIN_MACRO(4,  PB9, RCC_APB1, TIM_Channel_4, IRQn)

//Timer5
#define TIM5_CH1_PA0  TIMER_PIN_MACRO(5,  PA0, RCC_APB1, TIM_Channel_1, IRQn)
#define TIM5_CH2_PA1  TIMER_PIN_MACRO(5,  PA1, RCC_APB1, TIM_Channel_2, IRQn)
#define TIM5_CH3_PA2  TIMER_PIN_MACRO(5,  PA2, RCC_APB1, TIM_Channel_3, IRQn)
#define TIM5_CH4_PA3  TIMER_PIN_MACRO(5,  PA3, RCC_APB1, TIM_Channel_4, IRQn)

//Timer6
#define TIM6_CH1 TIMER_PERIPH_MACRO(6, RCC_APB1, TIM_Channel_1, DAC_IRQn)
#define TIM7_CH1 TIMER_PERIPH_MACRO(7, RCC_APB1, TIM_Channel_1, IRQn)

//------------------------------------------------------------------------------
// ADC

typedef struct {
    ADC_TypeDef* addr;
    uint8_t channel;
    uint32_t rcc_clock;
    RCC_ClockCmdFunction rcc_clock_cmd;
} ADC_Periph;

typedef struct {
    ADC_Periph adc;
    GPIO_Pin   gpio_pin;
} ADCPin;

#define ADC_PERIPH_MACRO(AdcNr, Channel, Rcc) { \
    .addr          = ADC##AdcNr,                \
    .channel       = ADC_##Channel,             \
    .rcc_clock     = Rcc##Periph_ADC##AdcNr,    \
    .rcc_clock_cmd = Rcc##PeriphClockCmd        \
}

#define ADC_PIN_MACRO(Pin, AdcNr, Channel, Rcc) {      \
    .adc      = ADC_PERIPH_MACRO(AdcNr, Channel, Rcc), \
    .gpio_pin = Pin,                                   \
}

// ADC1
#define ADC1_CH10 ADC_PIN_MACRO(PC0, 1, Channel_10, RCC_APB2)
#define ADC1_CH11 ADC_PIN_MACRO(PC1, 1, Channel_11, RCC_APB2)
#define ADC1_CH12 ADC_PIN_MACRO(PC2, 1, Channel_12, RCC_APB2)
#define ADC1_CH13 ADC_PIN_MACRO(PC3, 1, Channel_13, RCC_APB2)

// ADC2
#define ADC2_CH10 ADC_PIN_MACRO(PC0, 2, Channel_10, RCC_APB2)
#define ADC2_CH11 ADC_PIN_MACRO(PC1, 2, Channel_11, RCC_APB2)
#define ADC2_CH12 ADC_PIN_MACRO(PC2, 2, Channel_12, RCC_APB2)
#define ADC2_CH13 ADC_PIN_MACRO(PC3, 2, Channel_13, RCC_APB2)

// ADC3
#define ADC3_CH10 ADC_PIN_MACRO(PC0, 3, Channel_10, RCC_APB2)
#define ADC3_CH11 ADC_PIN_MACRO(PC1, 3, Channel_11, RCC_APB2)
#define ADC3_CH12 ADC_PIN_MACRO(PC2, 3, Channel_12, RCC_APB2)
#define ADC3_CH13 ADC_PIN_MACRO(PC3, 3, Channel_13, RCC_APB2)

//------------------------------------------------------------------------------
// DMA
typedef struct {
    DMA_Stream_TypeDef* stream;
    uint32_t rcc_clock;
    RCC_ClockCmdFunction rcc_clock_cmd;
    IRQn_Type irq_nr;
} DMA_Periph;

#define DMA_PERIPH_MACRO(Dma, Stream, Rcc) {\
    .stream         = Dma##_##Stream,       \
    .rcc_clock      = Rcc##Periph_##Dma,    \
    .rcc_clock_cmd  = Rcc##PeriphClockCmd,  \
    .irq_nr         = Dma##_##Stream##_IRQn \
}

// DMA2
#define DMA2_STREAM0 DMA_PERIPH_MACRO(DMA2, Stream0, RCC_AHB1)

//------------------------------------------------------------------------------
// CAN

typedef struct {
    CAN_TypeDef* addr;
    uint32_t rcc_clock;
    RCC_ClockCmdFunction rcc_clock_cmd;
} CAN_Periph;

typedef struct {
    CAN_Periph can;
    GPIO_Pin gpio_pin;
    uint8_t  gpio_af_can;
} CAN_TxPin;

typedef CAN_TxPin CAN_RxPin;

#define CAN_PIN_MACRO(Pin, Can, Rcc) {      \
    .gpio_pin       = Pin,                  \
    .gpio_af_can    = GPIO_AF_##Can,        \
    .can            = Can,                  \
    .rcc_clock      = Rcc##Periph_##Can,    \
    .rcc_clock_cmd  = Rcc##PeriphClockCmd,  \
}

// CAN 1
#define CAN1_RX_PB8   CAN_PIN_MACRO( PB8, CAN1, RCC_APB1)
#define CAN1_TX_PB9   CAN_PIN_MACRO( PB9, CAN1, RCC_APB1)
#define CAN1_RX_PA11  CAN_PIN_MACRO(PA11, CAN1, RCC_APB1)
#define CAN1_TX_PA12  CAN_PIN_MACRO(PA12, CAN1, RCC_APB1)

//CAN 2
#define CAN2_RX_PB5   CAN_PIN_MACRO( PB5, CAN2, RCC_APB1)
#define CAN2_TX_PB6   CAN_PIN_MACRO( PB6, CAN2, RCC_APB1)
#define CAN2_RX_PB12  CAN_PIN_MACRO(PB12, CAN2, RCC_APB1)
#define CAN2_TX_PB13  CAN_PIN_MACRO(PB13, CAN2, RCC_APB1)

#endif // TERA_LIBRARY_STM32F40_41XXX_DEF_H_INCLUDED
