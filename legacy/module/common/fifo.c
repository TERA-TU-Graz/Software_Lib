#include "fifo.h"

//-----------------------------------------------------------------------------
void FIFO_init(FifoQueue* queue , uint8_t* buffer, uint16_t length) {
  queue->read = 0;
  queue->write = 0;
  queue->mask = length-1;
  queue->buffer = buffer;
}

//-----------------------------------------------------------------------------
void FIFO_append(FifoQueue* queue, uint8_t value) {
  (queue->buffer)[queue->write] = value;
  ++(queue->write);
  queue->write &= (queue->mask);
}

//-----------------------------------------------------------------------------
bool FIFO_append2(FifoQueue* queue, uint8_t value) {
  bool ret = true;

  // überlauf feststellen, write ist eins hinter read
  if ((queue->read - queue->write)==1 ||
      (queue->read==0 && queue->write==queue->mask)) { // überlauf
    ret = false;
  } else {
    (queue->buffer)[queue->write] = value;
    ++(queue->write);
    queue->write &= (queue->mask);
  }
  return ret;
}

//-----------------------------------------------------------------------------
bool FIFO_fetch(FifoQueue* queue, uint8_t *value) {
  if (queue->read==queue->write) // fifo empty
    return false;

  *value = (queue->buffer)[queue->read];
  ++(queue->read);
  queue->read &= (queue->mask);

  return true;
}
