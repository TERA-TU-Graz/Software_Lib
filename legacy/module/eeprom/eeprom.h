#ifndef TERA_LIBRARY_EEPROM_H_INCLUDED
#define TERA_LIBRARY_EEPROM_H_INCLUDED

#include <module/common/modules_def.h>

// TODO EEPROM - this module needs better functions like taking a length parameter etc.
uint16_t EEPROM_init();
uint16_t EEPROM_readVariable(uint32_t address, uint16_t* data);
uint16_t EEPROM_writeVariable(uint32_t address, uint16_t data);
uint16_t EEPROM_format();

#endif // TERA_LIBRARY_EEPROM_H_INCLUDED
