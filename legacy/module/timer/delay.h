#ifndef TERA_LIBRARY_DELAY_H_INCLUDED
#define TERA_LIBRARY_DELAY_H_INCLUDED

#include <module/common/modules_def.h>


void DELAY_init();

void DELAY_wait(uint32_t delay_ms);
void DELAY_waitMicro(uint32_t delay_100us);

#endif // TERA_LIBRARY_DELAY_H_INCLUDED
