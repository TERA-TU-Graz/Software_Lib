#include "adc_dma.h"
#include <module/gpio/gpio.h>

//TODO ADC_DMA - allocating space for a probably unused channel is not nice
#define MAX_ADC_CHANNEL 18
#define ADC_STORAGE_INIT {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

#define BUFFERSIZE 32 //== number of samples to average

__attribute__ ((section (".sram"))) uint16_t ADC1_temp_storage[MAX_ADC_CHANNEL];
__attribute__ ((section (".sram"))) uint16_t ADC2_temp_storage[MAX_ADC_CHANNEL];
__attribute__ ((section (".sram"))) uint16_t ADC3_temp_storage[MAX_ADC_CHANNEL];

uint16_t ADC1_avg_storage[MAX_ADC_CHANNEL];
uint16_t ADC2_avg_storage[MAX_ADC_CHANNEL];
uint16_t ADC3_avg_storage[MAX_ADC_CHANNEL];

//------------------------------------------------------------------------------
// DMA interrupts
//

//TODO ADC_DMA - IRQ Handlers are currently hardcoded without selection option
void DMA2_Stream0_IRQHandler() {
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)) {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        static uint32_t ADC1_sum_storage[MAX_ADC_CHANNEL] = ADC_STORAGE_INIT;

        unsigned i = 0;
        for (i = 0; i < MAX_ADC_CHANNEL; i++) {
            ADC1_sum_storage[i] += ADC1_temp_storage[i];
        }

        static uint32_t buffercnt = 0;
        buffercnt = (buffercnt + 1) % BUFFERSIZE;
        if (buffercnt == 0) {
            for (i = 0; i < MAX_ADC_CHANNEL; i++) {
                ADC1_avg_storage[i] = ADC1_sum_storage[i] / BUFFERSIZE;
                ADC1_sum_storage[i] = 0;
            }
        }
    }
}

void DMA2_Stream1_IRQHandler() {
    if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1)) {
        DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
        static uint32_t ADC3_sum_storage[MAX_ADC_CHANNEL] = ADC_STORAGE_INIT;

        unsigned i = 0;
        for (i = 0; i < MAX_ADC_CHANNEL; i++) {
            ADC3_sum_storage[i] += ADC1_temp_storage[i];
        }

        static uint32_t buffercnt = 0;
        buffercnt = (buffercnt + 1) % BUFFERSIZE;
        if (buffercnt == 0) {
            for (i = 0; i < MAX_ADC_CHANNEL; i++) {
                ADC3_avg_storage[i] = ADC3_sum_storage[i] / BUFFERSIZE;
                ADC3_sum_storage[i] = 0;
            }
        }
    }
}

void DMA2_Stream3_IRQHandler() {
    if (DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3)) {
        DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);
        static uint32_t ADC2_sum_storage[MAX_ADC_CHANNEL] = ADC_STORAGE_INIT;

        unsigned i = 0;
        for (i = 0; i < MAX_ADC_CHANNEL; i++) {
            ADC2_sum_storage[i] += ADC1_temp_storage[i];
        }

        static uint32_t buffercnt = 0;
        buffercnt = (buffercnt + 1) % BUFFERSIZE;
        if (buffercnt == 0) {
            for (i = 0; i < MAX_ADC_CHANNEL; i++) {
                ADC2_avg_storage[i] = ADC2_sum_storage[i] / BUFFERSIZE;
                ADC2_sum_storage[i] = 0;
            }
        }
    }
}

//------------------------------------------------------------------------------
// Increments, then returns the number of used channels for the given ADC
// @param ADCx
// @return the increased value
uint32_t incInitializedChannelCount(ADC_TypeDef* ADCx) {
    static uint32_t adc1_channel_cnt = 0;
    static uint32_t adc2_channel_cnt = 0;
    static uint32_t adc3_channel_cnt = 0;

    uint32_t rank = 0;
    if (ADCx == ADC1) {
        rank = ++adc1_channel_cnt;
    } else if (ADCx == ADC2) {
        rank = ++adc2_channel_cnt;
    } else if (ADCx == ADC3) {
        rank = ++adc3_channel_cnt;
    } else {
        while (1)
            ; //configure when needed
    }

    return rank;
}

uint16_t* lookupTempDMAMemoryStorage(ADC_TypeDef* ADCx) {
    uint16_t* storage = 0;
    if (ADCx == ADC1) {
        storage = ADC1_temp_storage;
    } else if (ADCx == ADC2) {
        storage = ADC2_temp_storage;
    } else if (ADCx == ADC3) {
        storage = ADC3_temp_storage;
    } else {
        while (1)
            ; //configure when needed
    }

    return storage;
}

uint16_t* lookupAvgDMAMemoryStorage(ADC_TypeDef* ADCx) {
    uint16_t* storage = 0;
    if (ADCx == ADC1) {
        storage = ADC1_avg_storage;
    } else if (ADCx == ADC2) {
        storage = ADC2_avg_storage;
    } else if (ADCx == ADC3) {
        storage = ADC3_avg_storage;
    } else {
        while (1)
            ; //configure when needed
    }

    return storage;
}

uint16_t* ADC_initWithDMA(ADCPin* pin, DMA_Periph* dma, uint32_t dma_channel) {
    ADC_InitTypeDef adc_init;
    ADC_CommonInitTypeDef adc_common_init;
    DMA_InitTypeDef dma_init;
    GPIO_InitStruct gpio_init;

    /* Enable ADCx, DMA and GPIO clocks ***************************************/
    dma->rcc_clock_cmd(dma->rcc_clock, ENABLE);
    pin->gpio_pin.rcc_clock_cmd(pin->gpio_pin.rcc_clock, ENABLE);
    pin->adc.rcc_clock_cmd(pin->adc.rcc_clock, ENABLE);

    uint32_t rank = incInitializedChannelCount(pin->adc.addr);
    uint16_t* temp_storage_addr = lookupTempDMAMemoryStorage(pin->adc.addr);

    /* DMA2 Stream channel configuration **************************************/
    DMA_ITConfig(dma->stream, DMA_IT_TC, DISABLE);
    DMA_DeInit(dma->stream);

    dma_init.DMA_Channel            = dma_channel;
    dma_init.DMA_PeripheralBaseAddr = (uint32_t) &(pin->adc.addr->DR);
    dma_init.DMA_Memory0BaseAddr    = (uint32_t) temp_storage_addr;

    dma_init.DMA_DIR        = DMA_DIR_PeripheralToMemory;
    dma_init.DMA_BufferSize = rank; //storage_places_cnt;

    dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_init.DMA_MemoryInc     = DMA_MemoryInc_Enable;

    dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // ADC -> HalfWords
    dma_init.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;

    dma_init.DMA_Mode     = DMA_Mode_Circular; //no reinit after 1 transfer necessary
    dma_init.DMA_Priority = DMA_Priority_High;

    dma_init.DMA_FIFOMode        = DMA_FIFOMode_Disable;
    dma_init.DMA_FIFOThreshold   = DMA_FIFOThreshold_HalfFull; //Will be ignored in direct mode
    dma_init.DMA_MemoryBurst     = DMA_MemoryBurst_Single; //no burst transfers allowed in direct mode
    dma_init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(dma->stream, &dma_init);
    DMA_Cmd(dma->stream, ENABLE);

    /* Configure ADC Channel pin as analog input ******************************/
    gpio_init.mode = GPIO_MODE_AN;
    gpio_init.pull_mode = GPIO_PULL_NONE;
    gpio_init.speed = GPIO_SPEED_SLOWEST;
    GPIO_init(&pin->gpio_pin, &gpio_init);

    /* ADC Common Init ********************************************************/
    adc_common_init.ADC_Mode             = ADC_Mode_Independent;
    adc_common_init.ADC_Prescaler        = ADC_Prescaler_Div8;
    adc_common_init.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled; //only relevant in multi ADC mode
    adc_common_init.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInit(&adc_common_init);

    /* ADC Init ***************************************************************/
    adc_init.ADC_Resolution           = ADC_Resolution_12b;
    adc_init.ADC_ScanConvMode         = ENABLE;
    adc_init.ADC_ContinuousConvMode   = ENABLE;
    adc_init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc_init.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T1_CC1; //ignored (ADC_ExternalTrigConvEdge_None)
    adc_init.ADC_DataAlign            = ADC_DataAlign_Right;
    adc_init.ADC_NbrOfConversion      = rank; //storage_places_cnt;
    ADC_Init(pin->adc.addr, &adc_init);

    /* ADC regular channel configuration **************************************/
    ADC_RegularChannelConfig(pin->adc.addr, pin->adc.channel, rank,
            ADC_SampleTime_480Cycles);

    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(pin->adc.addr, ENABLE);

    /* Enable ADC DMA */
    ADC_DMACmd(pin->adc.addr, ENABLE);

    /* Enable ADC */
    ADC_Cmd(pin->adc.addr, ENABLE);

    /* Enable DMA Stream Transfer Complete interrupt */
    DMA_ITConfig(dma->stream, DMA_IT_TC, ENABLE);

    /* Enable the DMA Stream IRQ Channel */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = dma->irq_nr;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    uint16_t* avg_storage_addr = lookupAvgDMAMemoryStorage(pin->adc.addr);
    return &avg_storage_addr[rank - 1];
}
