#ifdef STM32F40_41xxx

#include "dac.h"
#include <stdint.h>

// das sind anscheinend die eintigen DAC output's
#define DAC_OUTPUT_DAC1_GPIO_PIN        GPIO_Pin_4
#define DAC_OUTPUT_DAC1_GPIO_PORT       GPIOA

#define DAC_OUTPUT_DAC2_GPIO_PIN        GPIO_Pin_5
#define DAC_OUTPUT_DAC2_GPIO_PORT       GPIOA

extern uint16_t dac_output_dac1_;
extern uint16_t dac_output_dac2_;

/**
 * 12bit Werte, von 0-4095
 * alle größeren Werte werden einfach abgeschnitten -> es werden die unteren 12bit dargestellt
 */
__attribute__ ((section (".sram"))) uint16_t dac_output_dac1_;
__attribute__ ((section (".sram"))) uint16_t dac_output_dac2_;

__attribute__((deprecated("This is an ugly 'module'")))
void DAC_OUTPUT_init() {
  DMA_InitTypeDef DMA_InitStructure;
  DAC_InitTypeDef  DAC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  // enable clock's
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // port config
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_InitStructure.GPIO_Pin = DAC_OUTPUT_DAC1_GPIO_PIN;
  GPIO_Init(DAC_OUTPUT_DAC1_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = DAC_OUTPUT_DAC2_GPIO_PIN;
  GPIO_Init(DAC_OUTPUT_DAC2_GPIO_PORT, &GPIO_InitStructure);

  // DAC config
  DAC_DeInit();

  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);


  // 84MHz
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = 0xFF; // ca. 3us, laut datenblatt benötigt der dac diese zeit um das signal auszugeben
  // keine grooße last anhängen -> oszi geht
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);


  DMA_DeInit(DMA1_Stream5);
  DMA_DeInit(DMA1_Stream6);

  DMA_InitStructure.DMA_Channel = DMA_Channel_7;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(DAC->DHR12R1);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&dac_output_dac1_;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);

  DMA_Cmd(DMA1_Stream5, ENABLE);
  DAC_Cmd(DAC_Channel_1, ENABLE);
  DAC_DMACmd(DAC_Channel_1, ENABLE);


  DMA_InitStructure.DMA_Channel = DMA_Channel_7;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(DAC->DHR12R2);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &dac_output_dac2_;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);

  DMA_Cmd(DMA1_Stream6, ENABLE);
  DAC_Cmd(DAC_Channel_2, ENABLE);
  DAC_DMACmd(DAC_Channel_2, ENABLE);


  dac_output_dac1_ = 0;
  dac_output_dac2_ = 0;

  TIM_Cmd(TIM6, ENABLE);
}

#endif // STM32F40_41xxx
