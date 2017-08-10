#pragma once

#include <config/adc_def.h>

//Adc_t TERA_ADC_init(AdcEnum_t adc, AdcConfig_t* cfg);
//Adc_t TERA_ADC_configure(AdcConfig_t* cfg);
//int TERA_ADC_initChannel(AdcChannel_t channel, Pin_t* pin);
//int TERA_ADC_initChannel(AdcChannel_t channel, Pin_t* pin, AdcConfig_t* cfg);
//// Locks the current configuration
//void TERA_ADC_lockConfig();

// @return The current resolution in bits
uint8_t TERA_ADC_getResolution(AdcChannel_t* channel);

// @return the sampling period of the ADC-channel in nanoseconds
//uint32_t TERA_ADC_getConversionTime(Adc_t adc);

uint32_t TERA_ADC_read(AdcChannel_t* channel);
//AdcResult_t TERA_ADC_read(AdcChannel_t channel, uint16_t* dst);
//AdcResult_t TERA_ADC_readNB(AdcChannel_t channel, uint16_t* dst);

// Set the conversion result. Should be called from the ISR to provide the
// conversion result
// @param AdcChannel*   Pointer to the ADC channel for which the conversion
//                      result shall be set
// @param uint32_t      The conversion result
void TERA_ADC_setConversionResult(AdcChannel_t* channel, uint32_t result);

