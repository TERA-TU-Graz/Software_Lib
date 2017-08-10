#ifndef TERA_LIBRARY_GPIO_H_INCLUDED
#define TERA_LIBRARY_GPIO_H_INCLUDED

#include <module/common/modules_def.h>

enum GPIO_PullMode {
    GPIO_PULL_DOWN, /*!< weak pull down resitor */
    GPIO_PULL_NONE, /*!< floating pin, no pull resistor */
    GPIO_PULL_UP    /*!< weak pull up resistor */
};

enum GPIO_Mode {
    GPIO_MODE_AF_OD,  /*!< Alternative function, open drain output*/
    GPIO_MODE_AF_PP,  /*!< Alternative function, push-pull output*/
    GPIO_MODE_AN,     /*!< Analog */
    GPIO_MODE_IN,     /*!< Digital input */
    GPIO_MODE_OUT_OD, /*!< Digital open drain output*/
    GPIO_MODE_OUT_PP, /*!< Digital push-pull output */
};

enum GPIO_Speed {
    GPIO_SPEED_SLOWEST, /*!< typically <5MHz */
    GPIO_SPEED_SLOW,    /*!< typically <20MHz */
    GPIO_SPEED_MEDIUM,  /*!< typically <50Mhz */
    GPIO_SPEED_FAST,    /*!< typically <75Mhz */
    GPIO_SPEED_FASTEST, /*!< typically >=75Mhz */
};

typedef struct {
    enum GPIO_Mode mode;
    enum GPIO_PullMode pull_mode;
    enum GPIO_Speed speed;
} GPIO_InitStruct;

//TODO GPIO - an init function which takes a default output value would be nice
void GPIO_init(GPIO_Pin* pin, GPIO_InitStruct* init);

uint8_t GPIO_read(GPIO_Periph* gpio);
uint8_t GPIO_readBit(GPIO_Pin* pin);
uint8_t GPIO_readOutput(GPIO_Periph* gpio);
uint8_t GPIO_readOutputBit(GPIO_Pin* pin);

void GPIO_write(GPIO_Periph* gpio, unsigned mask);
void GPIO_writeBit(GPIO_Pin* pin, unsigned bitval);
void GPIO_setBit(GPIO_Pin* pin);
void GPIO_clearBit(GPIO_Pin* pin);
void GPIO_toggleBit(GPIO_Pin* pin);

#endif // TERA_LIBRARY_GPIO_H_INCLUDED
