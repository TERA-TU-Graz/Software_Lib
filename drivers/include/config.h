#ifndef TERALIB_DRIVERS_INCLUDE_TERA_ASSERT_H_INCLUDED
#define TERALIB_DRIVERS_INCLUDE_TERA_ASSERT_H_INCLUDED

#if !defined(TERA_CUSTOM_ASSERTS)
# if defined(DEBUG)
#   define TERA_ASSERT(_expr) if(!(_expr)){do{}while(1);}
# else
#   define TERA_ASSERT(_expr) do{}while(0);
# endif
# define TERA_ASSERT_ADC(_expr) TERA_ASSERT(_expr)
# define TERA_ASSERT_GPIO(_expr) TERA_ASSERT(_expr)
# define TERA_ASSERT_UART(_expr) TERA_ASSERT(_expr)
# define TERA_ASSERT_CAN(_expr) TERA_ASSERT(_expr)
#else
#include <tera_custom_asserts.h>
#endif


#if defined(USE_RTOS_NONE)
# define TERA_USED_RTOS 0
#elif defined(USE_RTOS_FREERTOS)
# define TERA_USED_RTOS 1
#else
# error("Please define the used RTOS.")
#endif

#endif /* TERALIB_DRIVERS_INCLUDE_TERA_ASSERT_H_INCLUDED */
