#include <uvos.h>

#ifdef UVOS_INCLUDE_SDCARD

#include "dosfs.h"


/* Local variables */

/* For caching - this feature has to be explicitly enabled, as it isn't reentrant */
/* and requires to use the same buffer pointer whenever reading a file. */
static uint32_t last_sector;
static uint8_t caching_enabled   = 0;

void DFS_CachingEnabledSet(uint8_t enable)
{
	caching_enabled = enable;
	last_sector = 0xffffffff;
}


/**
* Converts directory name to canonical name
* (missing pendant to DFS_anonicalToDir)
* dest must point to a 13-byte buffer
*/
char *DFS_DirToCanonical(char *dest, char *src)
{
	uint8_t pos = 0;

	while( pos < 8 && src[pos] != ' ' ) {
		*dest++ = src[pos++];
	}

	if( src[8] != ' ' ) {
		*dest++ = '.';

		pos = 8;
		while( pos < 11 && src[pos] != ' ' ) {
			*dest++ = src[pos++];
		}
	}

	/* Terminate string */
	*dest = 0;

	return dest;
}



/**
* Read sector from SD Card
* Returns 0 OK, nonzero for any error
*/
uint32_t DFS_ReadSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count)
{
	/* Only allow access to single unit */
	if(unit != 0) {
		return 1;
	}

	/* According to README.txt, count is always 1 - check this! */
	if(count != 1) {
		return 2;
	}

	/* Cache: */
	if(caching_enabled && sector == last_sector) {
		/* we assume that sector is already in *buffer */
		/* since the user has to take care that the same buffer is used for file reads, this */
		/* feature has to be explicitly enabled with DFS_CachingEnabledSet(uint8_t enable) */
		return 0;
	}

	last_sector = sector;

	/* Forward to UVOS */
	int32_t status;
	if((status = UVOS_SDCARD_SectorRead(sector, buffer)) != 0) {
		/* Cannot access SD Card */
		return 3;
	}

	/* Success */
	return 0;
}

/**
* Write sector to SD Card
* Returns 0 OK, nonzero for any error
*/
uint32_t DFS_WriteSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count)
{
	/* Only allow access to single unit */
	if(unit != 0) {
		return 1;
	}

	/* According to README.txt, count is always 1 - check this! */
	if(count != 1) {
		return 2;
	}

	/* Invalidate cache */
	last_sector = 0xffffffff;

	/* Forward to UVOS */
	int32_t status;
	if((status = UVOS_SDCARD_SectorWrite(sector, buffer)) < 0) {
		/* Cannot access SD Card */
		return 3;
	}

	/* Success */
	return 0;
}

#endif // UVOS_INCLUDE_SDCARD
