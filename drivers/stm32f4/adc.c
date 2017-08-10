// Standard library
#include <stdbool.h>

// HAL library
#include <stm32f4xx_hal_adc.h>

// Tera library
#include <config.h>
#include "adc.h"
#include "config/adc_def.h"


uint8_t TERA_ADC_getResolution(AdcChannel_t* adc_channel) {
  TERA_ASSERT_ADC(adc_channel != NULL);
  int tmp = (adc_channel->adc->Instance->CR1 >> 24) & 3; // ADC_CR1_RES bits
  int res = 12 - (tmp << 1); // 12 - x*2 = {12, 10, 8, 6}
  TERA_ASSERT_ADC(res == 6 || res==8 || res==10 || res==12)
  return res;
}

//uint32_t TERA_ADC_getConversionTime(AdcChannel_t* channel) {
//  TERA_ASSERT_ADC(false);
//  //TODO(dschaffenrath): implement TERA_ADC_getConversionTime
//  return 42;
//}

#if defined(TERA_USED_RTOS) && TERA_USED_RTOS == TERA_USED_RTOS_NONE
uint32_t TERA_ADC_read(AdcChannel_t* channel) {
  while(channel->result_available == 0){
    // polling
  }
  // uint32_t read/write should be atomic on Cortex-M
  uint32_t result = channel->result;
  channel->result_available = 0;
  return result;
}

void TERA_ADC_setConversionResult(AdcChannel_t* channel, uint32_t result){
  // uint32_t read/write should be atomic on Cortex-M
  channel->result = result;
  channel->result_available = 1;
}
#else
#  error("Unknown RTOS specified")
#endif

//void TERA_ADC_handleIRQ(Adc_t* adc){
//  HAL_ADC_IRQHandler(adc);
//}

//void TERA_ADC_readChannelGroup(AdcChannelList_t* channel_list) {
//  //RFC(dschaffenrath) this probably doesn't make much sense. It should be handled by a configuration option.
//  TERA_ASSERT_ADC(false); //UNTESTED!!!
//  TERA_ASSERT_ADC(channel_list != NULL);
//
//  // Set the conversion
//  AdcChannelList_t* it = channel_list;
//  uint8_t channel_cnt = 1;
//  uint16_t sqr[3] = {0,0,0};
//  for (;  it != NULL && channel_cnt < 7; channel_cnt++, it = it->next) {
//    TERA_ASSERT_ADC(it->next==NULL || it->item->adc.Instance == it->next->item->adc.Instance);
//    sqr[2] |= ADC_SQR3_RK(it->item->channel_nr, channel_cnt);
//  }
//  for (; it != NULL && channel_cnt < 15; channel_cnt++, it = it->next) {
//    TERA_ASSERT_ADC(it->next==NULL || it->item->adc.Instance == it->next->item->adc.Instance);
//    sqr[1] |= ADC_SQR2_RK(it->item->channel_nr, channel_cnt);
//  }
//  for (; it != NULL && channel_cnt < 20; channel_cnt++, it = it->next) {
//    TERA_ASSERT_ADC(it->next==NULL || it->item->adc.Instance == it->next->item->adc.Instance);
//    sqr[0] |= ADC_SQR1_RK(it->item->channel_nr, channel_cnt);
//  }
//  sqr[0] |= ADC_SQR1(channel_cnt);
//  TERA_ASSERT_ADC(channel_cnt <= 20);
//  ADC_TypeDef* adc = channel_list->item->adc.Instance;
//  adc->SQR3 |= sqr[2];
//  adc->SQR2 |= sqr[1];
//  adc->SQR1 |= sqr[0];
//
//  HAL_ADC_Start(channel_list->item->adc);
//  HAL_ADC_PollForConversion(channel_list->item->adc, HAL_MAX_DELAY);
//}
