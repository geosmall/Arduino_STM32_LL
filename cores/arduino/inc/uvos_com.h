#ifndef UVOS_COM_H
#define UVOS_COM_H

#include <stdint.h> /* uint*_t */
#include <stdbool.h> /* bool */

typedef uint16_t ( *uvos_com_callback )( uint32_t context, uint8_t *buf, uint16_t buf_len, uint16_t *headroom, bool *task_woken );
// typedef void ( *uvos_com_callback_ctrl_line )( uint32_t context, uint32_t mask, uint32_t state );
typedef void ( *uvos_com_callback_baud_rate )( uint32_t context, uint32_t baud );

struct uvos_com_driver {
  // void     ( *init )( uint32_t id );
  void     ( *set_baud )( uint32_t id, uint32_t baud );
  // void     ( *set_ctrl_line )( uint32_t id, uint32_t mask, uint32_t state );
  void     ( *tx_start )( uint32_t id, uint16_t tx_bytes_avail );
  void     ( *rx_start )( uint32_t id, uint16_t rx_bytes_avail );
  void     ( *bind_rx_cb )( uint32_t id, uvos_com_callback rx_in_cb, uint32_t context );
  void     ( *bind_tx_cb )( uint32_t id, uvos_com_callback tx_out_cb, uint32_t context );
  // void     ( *bind_ctrl_line_cb )( uint32_t id, uvos_com_callback_ctrl_line ctrl_line_cb, uint32_t context );
  void     ( *bind_baud_rate_cb )( uint32_t id, uvos_com_callback_baud_rate baud_rate_cb, uint32_t context );
  uint32_t ( *available )( uint32_t id );
};

/* Control line definitions */
#define COM_CTRL_LINE_DTR_MASK 0x01
#define COM_CTRL_LINE_RTS_MASK 0x02

/* Public Functions */
extern int32_t UVOS_COM_Init( uint32_t *com_id, const struct uvos_com_driver *driver, uint32_t lower_id, uint8_t *rx_buffer, uint16_t rx_buffer_len, uint8_t *tx_buffer, uint16_t tx_buffer_len );
extern int32_t UVOS_COM_ChangeBaud( uint32_t com_id, uint32_t baud );
// extern int32_t UVOS_COM_SetCtrlLine( uint32_t com_id, uint32_t mask, uint32_t state );
// extern int32_t UVOS_COM_RegisterCtrlLineCallback( uint32_t usart_id, uvos_com_callback_ctrl_line ctrl_line_cb, uint32_t context );
extern int32_t UVOS_COM_RegisterBaudRateCallback( uint32_t usart_id, uvos_com_callback_baud_rate baud_rate_cb, uint32_t context );
extern int32_t UVOS_COM_SendCharNonBlocking( uint32_t com_id, char c );
extern int32_t UVOS_COM_SendChar( uint32_t com_id, char c );
extern int32_t UVOS_COM_SendBufferNonBlocking( uint32_t com_id, const uint8_t *buffer, uint16_t len );
extern int32_t UVOS_COM_SendBuffer( uint32_t com_id, const uint8_t *buffer, uint16_t len );
extern int32_t UVOS_COM_SendStringNonBlocking( uint32_t com_id, const char *str );
extern int32_t UVOS_COM_SendString( uint32_t com_id, const char *str );
extern int32_t UVOS_COM_SendFormattedStringNonBlocking( uint32_t com_id, const char *format, ... );
extern int32_t UVOS_COM_SendFormattedString( uint32_t com_id, const char *format, ... );
extern uint16_t UVOS_COM_ReceiveBuffer( uint32_t com_id, uint8_t *buf, uint16_t buf_len, uint32_t timeout_ms );
extern int32_t UVOS_COM_ReceiveChar( uint32_t com_id, char *c );
extern int32_t UVOS_COM_RX_Data_Available ( uint32_t com_id );
extern uint32_t UVOS_COM_Available( uint32_t com_id );

#define COM_AVAILABLE_NONE (0)
#define COM_AVAILABLE_RX   (1 << 0)
#define COM_AVAILABLE_TX   (1 << 1)
#define COM_AVAILABLE_RXTX (COM_AVAILABLE_RX | COM_AVAILABLE_TX)

#endif /* UVOS_COM_H */