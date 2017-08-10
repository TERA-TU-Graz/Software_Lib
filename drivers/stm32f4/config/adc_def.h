#ifndef TERALIB_DRIVERS_STM32F4_ADC_DEF_H_INCLUDED
#define TERALIB_DRIVERS_STM32F4_ADC_DEF_H_INCLUDED

#include "stm32f4xx_hal_adc.h"

typedef ADC_HandleTypeDef Adc_t;
//typedef enum AdcChannelEnum {
//    kChannel0 = ADC_CHANNEL_0,
//    kChannel1 = ADC_CHANNEL_1,
//    kChannel2 = ADC_CHANNEL_2,
//    kChannel3 = ADC_CHANNEL_3,
//    kChannel4 = ADC_CHANNEL_4,
//    kChannel5 = ADC_CHANNEL_5,
//    kChannel6 = ADC_CHANNEL_6,
//    kChannel7 = ADC_CHANNEL_7,
//    kChannel8 = ADC_CHANNEL_8,
//    kChannel9 = ADC_CHANNEL_9,
//    kChannel10 = ADC_CHANNEL_10,
//    kChannel11 = ADC_CHANNEL_11,
//    kChannel12 = ADC_CHANNEL_12,
//    kChannel13 = ADC_CHANNEL_13,
//    kChannel14 = ADC_CHANNEL_14,
//    kChannel15 = ADC_CHANNEL_15,
//    kChannel16 = ADC_CHANNEL_16,
//    kChannel17 = ADC_CHANNEL_17,
//    kChannel18 = ADC_CHANNEL_18,
//} AdcChannelEnum_t;
typedef struct AdcChannel {
  Adc_t* adc;
//  AdcChannelEnum_t channel;
  uint32_t result;
  uint8_t channel_nr;
  volatile uint8_t result_available;
} AdcChannel_t;
//typedef struct AdcChannelList{
//  AdcChannel_t* item;
//  struct AdcChannelList* next;
//} AdcChannelList_t;

//typedef uint16_t AdcResult_t;


#endif // TERALIB_DRIVERS_STM32F4_ADC_DEF_H_INCLUDED
