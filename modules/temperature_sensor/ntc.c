
// standard library
#include <stdbool.h>
#include <math.h>

// Teralib drivers
#include <adc.h>
#include <config.h>

// Teralib modules
#include "ntc.h"


uint32_t TERA_NTC_read(__attribute((unused)) Ntc_t* ntc){
  TERA_ASSERT(false);
  return 0;
}

Ntc_t TERA_NTC_init(uint32_t constant_A1, uint32_t constant_B1){
  Ntc_t ntc = {
      .constant_A1 = constant_A1,
      .constant_B1 = constant_B1,
  };
  return ntc;
}

TemperatureSensor_t TERA_NTC_asTemperatureSensor(Ntc_t* ntc){
  TemperatureSensor_t sensor = {
      .read = (uint32_t (*)(TemperatureSensor_t*))TERA_NTC_read,
      ._sensor = (void*)ntc,
  };
  return sensor;
}

