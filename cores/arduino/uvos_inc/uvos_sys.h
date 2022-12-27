#pragma once

#define UVOS_SYS_SERIAL_NUM_BINARY_LEN 12
#define UVOS_SYS_SERIAL_NUM_ASCII_LEN  (UVOS_SYS_SERIAL_NUM_BINARY_LEN * 2)

/* Public Functions */
extern void UVOS_SYS_Init(void);
extern int32_t UVOS_SYS_Reset(void);
extern uint32_t UVOS_SYS_getCPUFlashSize(void);
extern int32_t UVOS_SYS_SerialNumberGetBinary(uint8_t array[UVOS_SYS_SERIAL_NUM_BINARY_LEN]);
extern int32_t UVOS_SYS_SerialNumberGet(char str[UVOS_SYS_SERIAL_NUM_ASCII_LEN + 1]);
