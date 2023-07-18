#include <uvos.h>
#include "imugyroaccelsettings.h"

static IMUGyroAccelSettingsData mpuSettings = { 0, 0, 0, false };

/**
 * @brief Updates IMU config based on provided IMUGyroAccelSettingsData object
 * @returns 0 if succeed or -1 otherwise
 */
void IMUGyroAccelSettingsGet( IMUGyroAccelSettingsData *dataOut )
{
	UVOS_Assert( dataOut );

	dataOut->GyroScale = mpuSettings.GyroScale;
	dataOut->AccelScale = mpuSettings.AccelScale;
	dataOut->FilterSetting = mpuSettings.FilterSetting;
	dataOut->isSet = mpuSettings.isSet;
}

void IMUGyroAccelSettingsSet( const IMUGyroAccelSettingsData *dataIn )
{
	UVOS_Assert( dataIn );

	mpuSettings.GyroScale = dataIn->GyroScale;
	mpuSettings.AccelScale = dataIn->AccelScale;
	mpuSettings.FilterSetting = dataIn->FilterSetting;
	mpuSettings.isSet = true;
}