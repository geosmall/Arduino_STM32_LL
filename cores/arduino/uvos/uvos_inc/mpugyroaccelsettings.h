#ifndef MPUGYROACCELSETTINGS_H
#define MPUGYROACCELSETTINGS_H

#include <stdbool.h>

/* Object constants */
#define MPUGYROACCELSETTINGS_OBJID 0x7E2826C8
#define MPUGYROACCELSETTINGS_ISSINGLEINST 1
#define MPUGYROACCELSETTINGS_ISSETTINGS 1
#define MPUGYROACCELSETTINGS_ISPRIORITY 0
#define MPUGYROACCELSETTINGS_NUMBYTES sizeof(MPUGyroAccelSettingsData)

/* Generic interface functions */
// int32_t MPUGyroAccelSettingsInitialize();
// UAVObjHandle MPUGyroAccelSettingsHandle();
// void MPUGyroAccelSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

/* Field GyroScale information */

// Enumeration options for field GyroScale
typedef enum __attribute__ ( ( __packed__ ) )
{
  MPUGYROACCELSETTINGS_GYROSCALE_SCALE_250 = 0,
  MPUGYROACCELSETTINGS_GYROSCALE_SCALE_500 = 1,
  MPUGYROACCELSETTINGS_GYROSCALE_SCALE_1000 = 2,
  MPUGYROACCELSETTINGS_GYROSCALE_SCALE_2000 = 3 /** default **/
} MPUGyroAccelSettingsGyroScaleOptions;

/* Field AccelScale information */

// Enumeration options for field AccelScale
typedef enum __attribute__ ( ( __packed__ ) )
{
  MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_2G = 0,
  MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_4G = 1,
  MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_8G = 2, /** default **/
  MPUGYROACCELSETTINGS_ACCELSCALE_SCALE_16G = 3
} MPUGyroAccelSettingsAccelScaleOptions;

/* Field FilterSetting information */

// Enumeration options for field FilterSetting
typedef enum __attribute__ ( ( __packed__ ) )
{
  MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_256_HZ = 0, /** default **/
  MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_188_HZ = 1,
  MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_98_HZ = 2,
  MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_42_HZ = 3,
  MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_20_HZ = 4,
  MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_10_HZ = 5,
  MPUGYROACCELSETTINGS_FILTERSETTING_LOWPASS_5_HZ = 6
} MPUGyroAccelSettingsFilterSettingOptions;




/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
  MPUGyroAccelSettingsGyroScaleOptions GyroScale;
  MPUGyroAccelSettingsAccelScaleOptions AccelScale;
  MPUGyroAccelSettingsFilterSettingOptions FilterSetting;
  bool isSet;

} __attribute__( ( packed ) ) MPUGyroAccelSettingsDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage faults
 * on Cortex M4F during load/store of float UAVO fields
 */
typedef MPUGyroAccelSettingsDataPacked __attribute__( ( aligned( 4 ) ) ) MPUGyroAccelSettingsData;

/* Typesafe Object access functions */
// static inline int32_t MPUGyroAccelSettingsGet(MPUGyroAccelSettingsData * dataOut) {
//     return UAVObjGetData(MPUGyroAccelSettingsHandle(), dataOut);
// }
// static inline int32_t MPUGyroAccelSettingsSet(const MPUGyroAccelSettingsData * dataIn) {
//     return UAVObjSetData(MPUGyroAccelSettingsHandle(), dataIn);
// }

/* Typesafe Object access functions */
extern void MPUGyroAccelSettingsGet( MPUGyroAccelSettingsData *dataOut );

extern void MPUGyroAccelSettingsSet( const MPUGyroAccelSettingsData *dataIn );

#endif /* MPUGYROACCELSETTINGS_H */

