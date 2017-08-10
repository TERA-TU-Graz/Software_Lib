#pragma once

#include <adc.h>

typedef struct TemperatureSensor{
  uint32_t (*read)(struct TemperatureSensor*);
  void* _sensor;
} TemperatureSensor_t;



uint32_t TERA_TEMPERATURE_SENSOR_read(TemperatureSensor_t* sensor){
  return sensor->read(sensor->_sensor);
}
