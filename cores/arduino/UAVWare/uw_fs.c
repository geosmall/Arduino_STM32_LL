// #include <UAVWare.h>
#include <uvos.h>
#include <uw_fs.h>

static UW_fs_type_t UW_fs_type = FS_TYPE_INVALID;
static UW_fs_file_t UW_fs_file;
static int fres;

// Initialize logical file system
// Returns 0 on success, or -1 on failure
int UW_fs_init( void )
{
#if defined ( UVOS_INCLUDE_SDCARD ) // prefer SD Card first
	UW_fs_type = FS_TYPE_FATFS; // SD Card uses FatFs
#elif defined ( UVOS_INCLUDE_FLASH )
	UW_fs_type = FS_TYPE_LITTLEFS; // SPI Flash uses LittleFS
#else
	UW_fs_type = FS_TYPE_INVALID;
#endif // defined ( UVOS_INCLUDE_SDCARD )

	// Check if target has a valid storage means
	if ( UW_fs_type == FS_TYPE_FATFS ) {
		// Make sure SD Card FatFs has been mounted (see target board.c)
		fres = UVOS_SDCARD_IsMounted();
		if ( !fres ) {
			return -1;
		}
	} else if ( UW_fs_type == FS_TYPE_LITTLEFS ) {
		return -1;
	}

	return 0;
}

// Get the mounted FS type
// Returns FS_TYPE_INVALID = 0, FS_TYPE_FATFS = 1 or FS_TYPE_LITTLEFS = 2
UW_fs_type_t UW_fs_get_type( void )
{
	return UW_fs_type;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

// Get volume info (total and free Kbytes)
// Returns 0 on success, or -1 on failure
int UW_fs_get_vol_info( UW_fs_vol_info_t *vol_info )
{
	if ( UW_fs_type == FS_TYPE_FATFS ) {
		SDCARD_vol_info_t sdcard_info;
		fres = UVOS_SDCARD_GetVolInfo( &sdcard_info );
		if ( fres ) {
			return -1;
		}
		vol_info->vol_total_Kbytes = sdcard_info.vol_total_Kbytes;
		vol_info->vol_free_Kbytes = sdcard_info.vol_free_Kbytes;
	} else if ( UW_fs_type == FS_TYPE_LITTLEFS ) {
		return -1;
	}
	return 0;
}

#pragma GCC pop_options

// Open a file in a given mode per fopen_mode_t (from uw_fs.h)
// Returns 0 if file open is successful, -1 if unsuccessful
int UW_fs_open( UW_fs_file_t *file, const char *path, fopen_mode_t mode )
{
	if ( UW_fs_type == FS_TYPE_FATFS ) {
		file->type = FS_TYPE_FATFS;
		fres = UVOS_SDCARD_Open( &file->fatfile, path, mode );
		if ( fres ) {
			return -1;
		}
	} else if ( UW_fs_type == FS_TYPE_LITTLEFS ) {
		return -1;
	}
	return 0;
}

// Read data from a file into a buffer
// Returns the number of bytes read, or -1 on failure
int UW_fs_read( UW_fs_file_t *file, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read )
{
	if ( UW_fs_type == FS_TYPE_FATFS ) {
		// UVOS_Assert( file->type == FS_TYPE_FATFS );
		fres = UVOS_SDCARD_Read( &file->fatfile, buf, bytes_to_read, bytes_read );
		if ( fres ) {
			return -1;
		}
	} else if ( UW_fs_type == FS_TYPE_LITTLEFS ) {
		return -1;
	}
	return 0;
}

// Write data from a buffer into a file
// Returns the number of bytes written, or -1 on failure
int UW_fs_write( UW_fs_file_t *file, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written )
{
	if ( UW_fs_type == FS_TYPE_FATFS ) {
		// UVOS_Assert( file->type == FS_TYPE_FATFS );
		fres = UVOS_SDCARD_Write( &file->fatfile, buf, bytes_to_write, bytes_written );
		if ( fres ) {
			return -1;
		}
	} else if ( UW_fs_type == FS_TYPE_LITTLEFS ) {
		return -1;
	}
	return 0;
}

// Seek to a position in a file
// Returns 0 on success, or -1 on failure
int UW_fs_seek( UW_fs_file_t *file, uint32_t offset )
{
	if ( UW_fs_type == FS_TYPE_FATFS ) {
		// UVOS_Assert( file->type == FS_TYPE_FATFS );
		fres = UVOS_SDCARD_Seek( &file->fatfile, offset );
		if ( fres ) {
			return -1;
		}
	} else if ( UW_fs_type == FS_TYPE_LITTLEFS ) {
		return -1;
	}
	return 0;
}

// Get the current position in a file
// Returns the position, or -1 on failure
uint32_t UW_fs_tell( UW_fs_file_t *file )
{
	if ( UW_fs_type == FS_TYPE_FATFS ) {
		// UVOS_Assert( file->type == FS_TYPE_FATFS );
		return UVOS_SDCARD_Tell( &file->fatfile );
	} else if ( UW_fs_type == FS_TYPE_LITTLEFS ) {
		return -1;
	}
	return 0;
}

// Close a file
// Returns 0 on success, or -1 on failure
int UW_fs_close( UW_fs_file_t *file )
{
	if ( UW_fs_type == FS_TYPE_FATFS ) {
		// UVOS_Assert( file->type == FS_TYPE_FATFS );
		fres = UVOS_SDCARD_Close( &file->fatfile );
		if ( fres ) {
			return -1;
		}
	} else if ( UW_fs_type == FS_TYPE_LITTLEFS ) {
		return -1;
	}
	return 0;
}

// Delete a file
// Returns 0 on success, or -1 on failure
int UW_fs_remove( const char *path )
{
	if ( UW_fs_type == FS_TYPE_FATFS ) {
		// UVOS_Assert( file->type == FS_TYPE_FATFS );
		fres = UVOS_SDCARD_Remove( path );
		if ( fres ) {
			return -1;
		}
	} else if ( UW_fs_type == FS_TYPE_LITTLEFS ) {
		return -1;
	}
	return 0;
}