#ifndef UVOS_MPU9250_CONFIG_H
#define UVOS_MPU9250_CONFIG_H

#include "imugyroaccelsettings.h"
#include "uvos_mpu9250.h"

#define UVOS_MPU9250_CONFIG_MAP_GYROSCALE(x) \
  (x == IMUGYROACCELSETTINGS_GYROSCALE_SCALE_250 ? UVOS_MPU9250_SCALE_250_DEG : \
   x == IMUGYROACCELSETTINGS_GYROSCALE_SCALE_500 ? UVOS_MPU9250_SCALE_500_DEG : \
   x == IMUGYROACCELSETTINGS_GYROSCALE_SCALE_1000 ? UVOS_MPU9250_SCALE_1000_DEG : \
   UVOS_MPU9250_SCALE_2000_DEG)

#define UVOS_MPU9250_CONFIG_MAP_ACCELSCALE(x) \
  (x == IMUGYROACCELSETTINGS_ACCELSCALE_SCALE_2G ? UVOS_MPU9250_ACCEL_2G : \
   x == IMUGYROACCELSETTINGS_ACCELSCALE_SCALE_4G ? UVOS_MPU9250_ACCEL_4G : \
   x == IMUGYROACCELSETTINGS_ACCELSCALE_SCALE_16G ? UVOS_MPU9250_ACCEL_16G : \
   UVOS_MPU9250_ACCEL_8G)

#define UVOS_MPU9250_CONFIG_MAP_FILTERSETTING(x) \
  (x == IMUGYROACCELSETTINGS_FILTERSETTING_HIGH3 ? UVOS_MPU9250_LOWPASS_256_HZ : \
   x == IMUGYROACCELSETTINGS_FILTERSETTING_HIGH2 ? UVOS_MPU9250_LOWPASS_256_HZ : \
   x == IMUGYROACCELSETTINGS_FILTERSETTING_HIGH1 ? UVOS_MPU9250_LOWPASS_256_HZ : \
   x == IMUGYROACCELSETTINGS_FILTERSETTING_MED2 ? UVOS_MPU9250_LOWPASS_188_HZ : \
   x == IMUGYROACCELSETTINGS_FILTERSETTING_MED1 ? UVOS_MPU9250_LOWPASS_98_HZ : \
   x == IMUGYROACCELSETTINGS_FILTERSETTING_LOW ? UVOS_MPU9250_LOWPASS_42_HZ : \
   UVOS_MPU9250_LOWPASS_256_HZ)
/**
 * @brief Updates MPU9250 config based on MPUGyroAccelSettings UAVO
 * @returns 0 if succeed or -1 otherwise
 */
// int32_t UVOS_MPU9250_CONFIG_Configure()
// {
//   MPUGyroAccelSettingsInitialize();
//   MPUGyroAccelSettingsData mpuSettings;
//   MPUGyroAccelSettingsGet( &mpuSettings );
//   return UVOS_MPU9250_ConfigureRanges(
//            UVOS_MPU9250_CONFIG_MAP_GYROSCALE( mpuSettings.GyroScale ),
//            UVOS_MPU9250_CONFIG_MAP_ACCELSCALE( mpuSettings.AccelScale ),
//            UVOS_MPU9250_CONFIG_MAP_FILTERSETTING( mpuSettings.FilterSetting )
//          );
// }

#endif /* UVOS_MPU9250_CONFIG_H */
