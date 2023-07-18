#include <uvos.h>
#include "mpugyroaccelsettings.h"

static MPUGyroAccelSettingsData mpuSettings = { 0, 0, 0, false };

/**
 * @brief Updates IMU config based on provided MPUGyroAccelSettingsData object
 * @returns 0 if succeed or -1 otherwise
 */
void MPUGyroAccelSettingsGet( MPUGyroAccelSettingsData *dataOut )
{
	UVOS_Assert( dataOut );

	dataOut->GyroScale = mpuSettings.GyroScale;
	dataOut->AccelScale = mpuSettings.AccelScale;
	dataOut->FilterSetting = mpuSettings.FilterSetting;
	dataOut->isSet = mpuSettings.isSet;
}

void MPUGyroAccelSettingsSet( const MPUGyroAccelSettingsData *dataIn )
{
	UVOS_Assert( dataIn );

	mpuSettings.GyroScale = dataIn->GyroScale;
	mpuSettings.AccelScale = dataIn->AccelScale;
	mpuSettings.FilterSetting = dataIn->FilterSetting;
	mpuSettings.isSet = true;
}