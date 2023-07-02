#ifndef UVOS_MPU6000_CONFIG_H
#define UVOS_MPU6000_CONFIG_H

#include "mpugyroaccelsettings.h"
#include "uvos_mpu6000.h"
#include <stdint.h>

#define UVOS_MPU6000_CONFIG_MAP_GYROSCALE(x) \
    (x == MPUGYROACCELSETTINGS_GYROSCALE_SCALE_250 ? UVOS_MPU6000_SCALE_250_DEG : \
     x == MPUGYROACCELSETTINGS_GYROSCALE_SCALE_500 ? UVOS_MPU6000_SCALE_500_DEG : \
     x == MPUGYROACCELSETTINGS_GYROSCALE_SCALE_1000 ? UVOS_MPU6000_SCALE_1000_DEG : \
     UVOS_MPU6000_SCALE_2000_DEG)

#define UVOS_MPU6000_CONFIG_MAP_ACCELSCALE(x) \
    (x == MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_2G ? UVOS_MPU6000_ACCEL_2G : \
     x == MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_4G ? UVOS_MPU6000_ACCEL_4G : \
     x == MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_16G ? UVOS_MPU6000_ACCEL_16G : \
     UVOS_MPU6000_ACCEL_8G)

#define UVOS_MPU6000_CONFIG_MAP_FILTERSETTING(x) \
    (x == MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_188_HZ ? UVOS_MPU6000_LOWPASS_188_HZ : \
     x == MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_98_HZ ? UVOS_MPU6000_LOWPASS_98_HZ : \
     x == MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_42_HZ ? UVOS_MPU6000_LOWPASS_42_HZ : \
     x == MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_20_HZ ? UVOS_MPU6000_LOWPASS_20_HZ : \
     x == MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_10_HZ ? UVOS_MPU6000_LOWPASS_10_HZ : \
     x == MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_5_HZ ? UVOS_MPU6000_LOWPASS_5_HZ : \
     UVOS_MPU6000_LOWPASS_256_HZ)
/**
 * @brief Updates MPU6000 config based on Mpu6000Settings UAVO
 * @returns 0 if succeed or -1 otherwise
 */
static inline void MPUGyroAccelSettingsGet( MPUGyroAccelSettingsData *mpuSettings )
{
  mpuSettings->GyroScale = MPUGYROACCELSETTINGS_GYROSCALE_SCALE_2000;
  mpuSettings->AccelScale = MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_8G;
  mpuSettings->FilterSetting = MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_188_HZ;
}

/**
 * @brief Updates MPU6000 config based on Mpu6000Settings UAVO
 * @returns 0 if succeed or -1 otherwise
 */
int32_t UVOS_MPU6000_CONFIG_Configure()
{
  // MPUGyroAccelSettingsInitialize();
  MPUGyroAccelSettingsData mpuSettings;
  MPUGyroAccelSettingsGet( &mpuSettings );
  return UVOS_MPU6000_ConfigureRanges(
           UVOS_MPU6000_CONFIG_MAP_GYROSCALE( mpuSettings.GyroScale ),
           UVOS_MPU6000_CONFIG_MAP_ACCELSCALE( mpuSettings.AccelScale ),
           UVOS_MPU6000_CONFIG_MAP_FILTERSETTING( mpuSettings.FilterSetting )
         );
}

#endif /* UVOS_MPU6000_CONFIG_H */