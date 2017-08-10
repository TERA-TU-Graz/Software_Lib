#ifndef TERA_LIBRARY_STM32F10X_MD_DEF_H_INCLUDED
#define TERA_LIBRARY_STM32F10X_MD_DEF_H_INCLUDED

#include "stm32f10x.h"

typedef void (*RCC_ClockCmdFunction)(uint32_t, FunctionalState);

//-------------------------------------------------------------------------------------------------
// GPIO
// Type: GPIO_MyDef
//-------------------------------------------------------------------------------------------------
//
typedef struct
{
  GPIO_TypeDef* gpiox_;
  uint16_t      gpio_pin_x_;
  uint16_t      gpio_pinsourcex_;

  uint32_t             rcc_clock_;
  RCC_ClockCmdFunction rcc_clock_cmd_;
} GPIO_Pin;

#define MY_GPIO_MACRO(port, pin) {\
	GPIO##port, \
	GPIO_Pin_##pin, \
	GPIO_PinSource##pin, \
	RCC_APB2Periph_GPIO##port, \
	RCC_APB2PeriphClockCmd }

#define NONE  {0,0,0}

//PA0 - PA15
#define PA0   MY_GPIO_MACRO(A,0)
#define PA1   MY_GPIO_MACRO(A,1)
#define PA2   MY_GPIO_MACRO(A,2)
#define PA3   MY_GPIO_MACRO(A,3)
#define PA4   MY_GPIO_MACRO(A,4)
#define PA5   MY_GPIO_MACRO(A,5)
#define PA6   MY_GPIO_MACRO(A,6)
#define PA7   MY_GPIO_MACRO(A,7)
#define PA8   MY_GPIO_MACRO(A,8)
#define PA9   MY_GPIO_MACRO(A,9)
#define PA10  MY_GPIO_MACRO(A,10)
#define PA11  MY_GPIO_MACRO(A,11)
#define PA12  MY_GPIO_MACRO(A,12)
#define PA13  MY_GPIO_MACRO(A,13)
#define PA14  MY_GPIO_MACRO(A,14)
#define PA15  MY_GPIO_MACRO(A,15)

//PB0 - PB15
#define PB0   MY_GPIO_MACRO(B,0)
#define PB1   MY_GPIO_MACRO(B,1)
#define PB2   MY_GPIO_MACRO(B,2)
#define PB3   MY_GPIO_MACRO(B,3)
#define PB4   MY_GPIO_MACRO(B,4)
#define PB5   MY_GPIO_MACRO(B,5)
#define PB6   MY_GPIO_MACRO(B,6)
#define PB7   MY_GPIO_MACRO(B,7)
#define PB8   MY_GPIO_MACRO(B,8)
#define PB9   MY_GPIO_MACRO(B,9)
#define PB10  MY_GPIO_MACRO(B,10)
#define PB11  MY_GPIO_MACRO(B,11)
#define PB12  MY_GPIO_MACRO(B,12)
#define PB13  MY_GPIO_MACRO(B,13)
#define PB14  MY_GPIO_MACRO(B,14)
#define PB15  MY_GPIO_MACRO(B,15)

//PC0 - PC15
#define PC0   MY_GPIO_MACRO(C,0)
#define PC1   MY_GPIO_MACRO(C,1)
#define PC2   MY_GPIO_MACRO(C,2)
#define PC3   MY_GPIO_MACRO(C,3)
#define PC4   MY_GPIO_MACRO(C,4)
#define PC5   MY_GPIO_MACRO(C,5)
#define PC6   MY_GPIO_MACRO(C,6)
#define PC7   MY_GPIO_MACRO(C,7)
#define PC8   MY_GPIO_MACRO(C,8)
#define PC9   MY_GPIO_MACRO(C,9)
#define PC10  MY_GPIO_MACRO(C,10)
#define PC11  MY_GPIO_MACRO(C,11)
#define PC12  MY_GPIO_MACRO(C,12)
#define PC13  MY_GPIO_MACRO(C,13)
#define PC14  MY_GPIO_MACRO(C,14)
#define PC15  MY_GPIO_MACRO(C,15)

//PD0 - PD15
#define PD1   MY_GPIO_MACRO(D,1)
#define PD2   MY_GPIO_MACRO(D,2)
#define PD3   MY_GPIO_MACRO(D,3)
#define PD4   MY_GPIO_MACRO(D,4)
#define PD5   MY_GPIO_MACRO(D,5)
#define PD6   MY_GPIO_MACRO(D,6)
#define PD7   MY_GPIO_MACRO(D,7)
#define PD8   MY_GPIO_MACRO(D,8)
#define PD9   MY_GPIO_MACRO(D,9)
#define PD10  MY_GPIO_MACRO(D,10)
#define PD11  MY_GPIO_MACRO(D,11)
#define PD12  MY_GPIO_MACRO(D,12)
#define PD13  MY_GPIO_MACRO(D,13)
#define PD14  MY_GPIO_MACRO(D,14)
#define PD15  MY_GPIO_MACRO(D,15)

//-------------------------------------------------------------------------------------------------
// Timer
// Type: Timer_MyDef
//-------------------------------------------------------------------------------------------------
//
typedef struct
{
  const GPIO_Pin gpio_struct_;
  TIM_TypeDef* timx_;
  uint32_t rcc_clock_;
  RCC_ClockCmdFunction rcc_clock_cmd_;
  IRQn_Type timx_irqn_;
  uint16_t tim_channel_x_;
} TimerPin;

#define MY_ADV_TIMER_MACRO(timer, clock) 		 TIM##timer, RCC_##clock##Periph_TIM##timer, RCC_##clock##PeriphClockCmd
#define MY_TIMER_MACRO(timer, clock) 	  		 TIM##timer, RCC_##clock##Periph_TIM##timer, RCC_##clock##PeriphClockCmd, TIM##timer##_IRQn
#define MY_BASIC_TIMER_MACRO(timer, clock, irqn) TIM##timer, RCC_##clock##Periph_TIM##timer, RCC_##clock##PeriphClockCmd, TIM##irqn##_IRQn

//Timer1, irqn can be a value of :
//TIM1_BRK_TIM9_IRQn       TIM1 Break interrupt and TIM9 global interrupt
//TIM1_UP_TIM10_IRQn       TIM1 Update Interrupt and TIM10 global interrupt
//TIM1_TRG_COM_TIM11_IRQn  TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
//TIM1_CC_IRQn             TIM1 Capture Compare Interrupt
//#define TIM1_CH2_PA9(irqn)  {PA9, MY_ADV_TIMER_MACRO(1,9,APB2), irqn, 2}

// Timer 2
#define TIM2_CH1_PA0  {PA0, MY_TIMER_MACRO(2,APB1), TIM_Channel_1}
#define TIM2_CH2_PA1  {PA1, MY_TIMER_MACRO(2,APB1), TIM_Channel_2}
#define TIM2_CH3_PA2  {PA2, MY_TIMER_MACRO(2,APB1), TIM_Channel_3}
#define TIM2_CH4_PA3  {PA3, MY_TIMER_MACRO(2,APB1), TIM_Channel_4}

// Timer 3
#define TIM3_CH1_PA6  {PA6, MY_TIMER_MACRO(3,APB1), TIM_Channel_1}
#define TIM3_CH2_PA7  {PA8, MY_TIMER_MACRO(3,APB1), TIM_Channel_2}
#define TIM3_CH3_PB0  {PA8, MY_TIMER_MACRO(3,APB1), TIM_Channel_3}
#define TIM3_CH4_PB1  {PA8, MY_TIMER_MACRO(3,APB1), TIM_Channel_4}

// Timer 4
#define TIM4_CH1_PB6  {PB6, MY_TIMER_MACRO(4,APB1), TIM_Channel_1}
#define TIM4_CH2_PB7  {PB7, MY_TIMER_MACRO(4,APB1), TIM_Channel_2}
#define TIM4_CH3_PB8  {PB8, MY_TIMER_MACRO(4,APB1), TIM_Channel_3}
#define TIM4_CH4_PB9  {PB9, MY_TIMER_MACRO(4,APB1), TIM_Channel_4}

//-------------------------------------------------------------------------------------------------
// DMA
// Type: DMA_MyDef
//-------------------------------------------------------------------------------------------------
//
typedef struct
{
	DMA_Channel_TypeDef* dmax_channely_;
  //DMA_Stream_TypeDef* dmax_streamy_;

  uint32_t rcc_clock_;
  RCC_ClockCmdFunction rcc_clock_cmd_;

  IRQn_Type dmax_streamy_irq_;
  uint32_t dma_it_tc_;
} DMA_Periph;
#define MY_DMA_MACRO(number, stream) {DMA##number##_Channel##stream, RCC_AHBPeriph_DMA##number, RCC_AHBPeriphClockCmd, DMA##number##_Channel##stream##_IRQn, DMA##number##_IT_TC##stream }

// DMA1
#define DMA1_CHANNEL1_INIT MY_DMA_MACRO(1,1)


//-------------------------------------------------------------------------------------------------
// ADC
//-------------------------------------------------------------------------------------------------
//
// Config
#define ADC_MAX_CHANNELS  10
#define ADC_MODULE_COUNT  2
#define ADC_BUFFERSIZE    32
#define ADC_CLOCK				  RCC_PCLK2_Div8

#define ADC1_DMA 					 &DMA1_CHANNEL1_
#define ADC1_DMA_IRQHandler DMA1_Channel1_IRQHandler

struct _ADC_MyDef_;
typedef void (*ADC_Hook)( const struct _ADC_MyDef_ *adc);

typedef struct
{
	const GPIO_Pin gpio_struct_;
	uint8_t adc_channel_x_;
} ADC_Channel_MyDef;

typedef struct
{
	uint16_t *temp_storage_;
	uint16_t avg_storage_[ADC_MAX_CHANNELS];
  uint32_t sum_storage_[ADC_MAX_CHANNELS];

	uint8_t channels_initialized_;
	ADC_Hook avg_hook_;
} ADC_Info_MyDef;

typedef struct _ADC_MyDef_
{
	ADC_TypeDef* adcx_;
	ADC_Info_MyDef* info_;
	const DMA_Periph* dma_;

  uint32_t 				  	 rcc_clock_;
  RCC_ClockCmdFunction rcc_clock_cmd_;

} ADCPin;

extern ADC_Info_MyDef ADC1_Info_;
extern ADC_Info_MyDef ADC2_Info_;
extern ADC_Info_MyDef ADC3_Info_;

// Type: ADC_Channel_MyDef
#define ADC_CH0_INIT {PA0, ADC_Channel_0}
#define ADC_CH1_INIT {PA1, ADC_Channel_1}
#define ADC_CH2_INIT {PA2, ADC_Channel_2}
#define ADC_CH3_INIT {PA3, ADC_Channel_3}
#define ADC_CH8_INIT {PB0, ADC_Channel_8}
#define ADC_CH9_INIT {PB1, ADC_Channel_9}

// Type: ADC_MyDef
#define ADC_1_INIT {ADC1, &ADC1_Info_, ADC1_DMA, RCC_APB2Periph_ADC1, RCC_APB2PeriphClockCmd}


//-------------------------------------------------------------------------------------------------
// CAN
//-------------------------------------------------------------------------------------------------
//
// IRQn
#define CAN1_RX_IRQn USB_LP_CAN1_RX0_IRQn
#define CAN1_TX_IRQn USB_HP_CAN1_TX_IRQn

//Interrupt Handler
#define CAN1_RX_IRQHandler USB_LP_CAN1_RX0_IRQHandler
#define CAN1_TX_IRQHandler USB_HP_CAN1_TX_IRQHandler

//Heartbeat timer
#define CAN_TIMER_IRQHandler TIM4_IRQHandler
#define CAN_TIMER &TIM4_CH1_PB6_

typedef struct
{
	const GPIO_Pin gpio_struct_;
  CAN_TypeDef* canx_;
  uint32_t rcc_apb1periph_canx_; // (RCC_APB1Periph_CAN1 || RCC_APB1Periph_CAN2)
  uint8_t nvci_irq_;
} CAN_TxPin;

typedef struct
{
	const GPIO_Pin gpio_struct_;
  CAN_TypeDef * canx_;
  uint32_t rcc_apb1periph_canx_; // (RCC_APB1Periph_CAN1 || RCC_APB1Periph_CAN2)
	uint32_t remap_;
	uint8_t nvci_irq_;
} CAN_RxPin;

#define MY_CAN_MACRO(can_nr) CAN##can_nr, RCC_APB1Periph_CAN##can_nr

// CAN 1, remapped to PB8/9
// Type: CANRx_MyDef
#define CAN1_RX_PB8  { PB8, MY_CAN_MACRO(1), GPIO_Remap1_CAN1, CAN1_RX_IRQn }
// Type: CANTx_MyDef
#define CAN1_TX_PB9  { PB9, MY_CAN_MACRO(1), CAN1_TX_IRQn }

#endif // TERA_LIBRARY_STM32F10X_MD_DEF_H_INCLUDED
