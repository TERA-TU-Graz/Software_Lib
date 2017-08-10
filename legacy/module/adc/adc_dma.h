#ifndef TERA_LIBRARY_ADC_DMA_H_INCLUDED
#define TERA_LIBRARY_ADC_DMA_H_INCLUDED

#include <module/common/modules_def.h>

//TODO ADC_DMA - resolution setting, not using DMA,... Maybe create separate DMA module

uint16_t* ADC_initWithDMA(ADCPin* adc_pin, DMA_Periph* dma,
        uint32_t dma_channel);

#endif // TERA_LIBRARY_ADC_DMA_H_INCLUDED
