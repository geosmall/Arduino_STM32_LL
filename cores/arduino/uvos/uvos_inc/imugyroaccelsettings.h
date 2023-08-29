#ifndef IMUGYROACCELSETTINGS_H
#define IMUGYROACCELSETTINGS_H

#include <stdbool.h>

/* Object constants */
#define IMUGYROACCELSETTINGS_OBJID 0x7E2826C8
#define IMUGYROACCELSETTINGS_ISSINGLEINST 1
#define IMUGYROACCELSETTINGS_ISSETTINGS 1
#define IMUGYROACCELSETTINGS_ISPRIORITY 0
#define IMUGYROACCELSETTINGS_NUMBYTES sizeof(IMUGyroAccelSettingsData)

/* Generic interface functions */
// int32_t IMUGyroAccelSettingsInitialize();
// UAVObjHandle IMUGyroAccelSettingsHandle();
// void IMUGyroAccelSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

/* Field GyroScale information */

// Enumeration options for field GyroScale
typedef enum __attribute__( ( __packed__ ) )
{
  IMUGYROACCELSETTINGS_GYROSCALE_SCALE_250 = 0,
  IMUGYROACCELSETTINGS_GYROSCALE_SCALE_500 = 1,
  IMUGYROACCELSETTINGS_GYROSCALE_SCALE_1000 = 2,
  IMUGYROACCELSETTINGS_GYROSCALE_SCALE_2000 = 3 /** default **/
} IMUGyroAccelSettingsGyroScaleOptions;

/* Field AccelScale information */

// Enumeration options for field AccelScale
typedef enum __attribute__( ( __packed__ ) )
{
  IMUGYROACCELSETTINGS_ACCELSCALE_SCALE_2G = 0,
  IMUGYROACCELSETTINGS_ACCELSCALE_SCALE_4G = 1,
  IMUGYROACCELSETTINGS_ACCELSCALE_SCALE_8G = 2, /** default **/
  IMUGYROACCELSETTINGS_ACCELSCALE_SCALE_16G = 3
} IMUGyroAccelSettingsAccelScaleOptions;

/* Field FilterSetting information */

// Enumeration options for field FilterSetting
// typedef enum __attribute__( ( __packed__ ) )
// {
//   IMUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_256_HZ = 0, /** default **/
//   IMUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_188_HZ = 1,
//   IMUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_98_HZ = 2,
//   IMUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_42_HZ = 3,
//   IMUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_20_HZ = 4,
//   IMUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_10_HZ = 5,
//   IMUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_5_HZ = 6
// } IMUGyroAccelSettingsFilterSettingOptions;

// Enumeration options for field FilterSetting
typedef enum __attribute__( ( __packed__ ) )
{
  IMUGYROACCELSETTINGS_FILTERSETTING_DEFAULT = 0, /** default, approx. 250-300Hz **/
  IMUGYROACCELSETTINGS_FILTERSETTING_HIGH3 = 1,
  IMUGYROACCELSETTINGS_FILTERSETTING_HIGH2 = 2,
  IMUGYROACCELSETTINGS_FILTERSETTING_HIGH1 = 3,
  IMUGYROACCELSETTINGS_FILTERSETTING_MED2 = 4,
  IMUGYROACCELSETTINGS_FILTERSETTING_MED1 = 5,
  IMUGYROACCELSETTINGS_FILTERSETTING_LOW = 6
} IMUGyroAccelSettingsFilterSettingOptions;


/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
  IMUGyroAccelSettingsGyroScaleOptions GyroScale;
  IMUGyroAccelSettingsAccelScaleOptions AccelScale;
  IMUGyroAccelSettingsFilterSettingOptions FilterSetting;
  bool isSet;

} __attribute__( ( packed ) ) IMUGyroAccelSettingsDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage faults
 * on Cortex M4F during load/store of float UAVO fields
 */
typedef IMUGyroAccelSettingsDataPacked __attribute__( ( aligned( 4 ) ) ) IMUGyroAccelSettingsData;

/* Typesafe Object access functions */
// static inline int32_t IMUGyroAccelSettingsGet(IMUGyroAccelSettingsData * dataOut) {
//     return UAVObjGetData(IMUGyroAccelSettingsHandle(), dataOut);
// }
// static inline int32_t IMUGyroAccelSettingsSet(const IMUGyroAccelSettingsData * dataIn) {
//     return UAVObjSetData(IMUGyroAccelSettingsHandle(), dataIn);
// }

/* Typesafe Object access functions */
extern void IMUGyroAccelSettingsGet( IMUGyroAccelSettingsData *dataOut );

extern void IMUGyroAccelSettingsSet( const IMUGyroAccelSettingsData *dataIn );

#endif /* IMUGYROACCELSETTINGS_H */

