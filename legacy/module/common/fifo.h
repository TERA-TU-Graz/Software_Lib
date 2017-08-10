#ifndef TERA_LIBRARY_FIFO_H_INCLUDED
#define TERA_LIBRARY_FIFO_H_INCLUDED

#include "module/common/modules_def.h"
#include <stdbool.h>

#define FIFO_QUEUE_isEmpty(queue) ((queue).read == (queue).write)

//-----------------------------------------------------------------------------
typedef struct _FifoQueue_ {
  uint16_t read;
  uint16_t write;
  uint16_t mask;

  uint8_t* buffer;
} FifoQueue;

//-----------------------------------------------------------------------------
// length must be modulo 16
void FIFO_init(FifoQueue* queue , uint8_t* buffer, uint16_t length);

//-----------------------------------------------------------------------------
void FIFO_append(FifoQueue* queue, uint8_t value);

//-----------------------------------------------------------------------------
bool FIFO_append2(FifoQueue* queue, uint8_t value);

//-----------------------------------------------------------------------------
// returns false if no byte received
bool FIFO_fetch(FifoQueue* queue, uint8_t *value);


#endif // TERA_LIBRARY_FIFO_H_INCLUDED
