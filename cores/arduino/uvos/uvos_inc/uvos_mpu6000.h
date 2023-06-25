#ifndef UVOS_MPU6000_H
#define UVOS_MPU6000_H

#include "uvos.h"
#include "uvos_mpu60x0.h"

/* Public Functions */
extern int32_t UVOS_MPU6000_Init(uint32_t spi_id, uint32_t slave_num, const struct uvos_mpu60x0_cfg *new_cfg);
extern int32_t UVOS_MPU6000_Test();
extern void UVOS_MPU6000_SetGyroRange(enum uvos_mpu60x0_range);

#if defined(UVOS_MPU6000_ACCEL)
extern void UVOS_MPU6000_SetAccelRange(enum uvos_mpu60x0_accel_range);
#endif /* UVOS_MPU6000_ACCEL */

extern void UVOS_MPU6000_SetSampleRate(uint16_t samplerate_hz);
extern void UVOS_MPU6000_SetLPF(enum uvos_mpu60x0_filter filter);
extern bool UVOS_MPU6000_IRQHandler(void);

#endif /* UVOS_MPU6000_H */

