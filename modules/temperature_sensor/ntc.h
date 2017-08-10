#ifndef TERALIB_MODULES_TEMPERATURE_SENSOR_NTC_H_INCLUDED
#define TERALIB_MODULES_TEMPERATURE_SENSOR_NTC_H_INCLUDED

#include <adc.h>
#include "temperature_sensor.h"

typedef struct Ntc{
  uint32_t constant_A1;
  uint32_t constant_B1;
} Ntc_t;

Ntc_t TERA_NTC_init(uint32_t constant_A1, uint32_t constant_B1);
uint32_t TERA_NTC_read(Ntc_t* sensor);
TemperatureSensor_t TERA_NTC_asTemperatureSensor(Ntc_t* ntc);

#endif /* TERALIB_MODULES_TEMPERATURE_SENSOR_NTC_H_INCLUDED */
