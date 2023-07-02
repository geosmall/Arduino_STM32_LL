#ifndef _FIFO_BUFFER_H_
#define _FIFO_BUFFER_H_

#include "stdint.h"

// *********************

typedef struct {
  uint8_t *buf_ptr;
  volatile uint16_t rd;
  volatile uint16_t wr;
  uint16_t buf_size;
} t_fifo_buffer;

// *********************

uint16_t fifoBuf_getSize( t_fifo_buffer *buf );

uint16_t fifoBuf_getUsed( t_fifo_buffer *buf );
uint16_t fifoBuf_getFree( t_fifo_buffer *buf );

void fifoBuf_clearData( t_fifo_buffer *buf );
void fifoBuf_removeData( t_fifo_buffer *buf, uint16_t len );

int16_t fifoBuf_getBytePeek( t_fifo_buffer *buf );
int16_t fifoBuf_getByte( t_fifo_buffer *buf );

uint16_t fifoBuf_getDataPeek( t_fifo_buffer *buf, void *data, uint16_t len );
uint16_t fifoBuf_getData( t_fifo_buffer *buf, void *data, uint16_t len );

uint16_t fifoBuf_putByte( t_fifo_buffer *buf, const uint8_t b );

uint16_t fifoBuf_putData( t_fifo_buffer *buf, const void *data, uint16_t len );

void fifoBuf_init( t_fifo_buffer *buf, const void *buffer, const uint16_t buffer_size );

#endif /* _FIFO_BUFFER_H_ */