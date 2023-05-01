// #include <UAVWare.h>
#include <uvos.h>
#include "uw_fs.h"

static uvos_fs_type_t UW_fs_type = FS_TYPE_INVALID;
// static uvos_fs_type_t UW_fs_file;
static int fres;

// extern lfs_t FS_lfs;

/* Provide a flash file system drivers, for either
	 FatFS based SD Card or LittleFS based serial flash */
static const struct uvos_fs_driver_t uw_fs_driver = {
#if defined( UVOS_INCLUDE_FLASH )
	.mount_fs = UVOS_SPIF_MountFS,
	.unmount_fs = UVOS_SPIF_UnmountFS,
	.is_mounted = UVOS_SPIF_IsMounted,
	.get_vol_info = UVOS_SPIF_GetVolInfo,
	.file_open = UVOS_SPIF_File_Open,
	.file_read = UVOS_SPIF_File_Read,
	.file_write = UVOS_SPIF_File_Write,
	.file_seek = UVOS_SPIF_File_Seek,
	.file_tell = UVOS_SPIF_File_Tell,
	.file_close = UVOS_SPIF_File_Close,
	.file_remove = UVOS_SPIF_File_Remove,
	.dir_open = UVOS_SPIF_Dir_Open,
	.dir_close = UVOS_SPIF_Dir_Close,
	.dir_read = UVOS_SPIF_Dir_Read,
#elif defined( UVOS_INCLUDE_SDCARD )
	.mount_fs = UVOS_SDCARD_MountFS,
	.unmount_fs = UVOS_SDCARD_UnmountFS,
	.is_mounted = UVOS_SDCARD_IsMounted,
	.get_vol_info = UVOS_SDCARD_GetVolInfo,
	.file_open = UVOS_SDCARD_Open,
	.file_read = UVOS_SDCARD_Read,
	.file_write = UVOS_SDCARD_Write,
	.file_seek = UVOS_SDCARD_Seek,
	.file_tell = UVOS_SDCARD_Tell,
	.file_close = UVOS_SDCARD_Close,
	.file_remove = UVOS_SDCARD_Remove,
	// .dir_open = UVOS_SDCARD_Open,
	// .dir_close = UVOS_SDCARD_Close,
	// .dir_read = UVOS_SDCARD_Read,
#else
#error No SPI based storage defined
#endif // defined( UVOS_INCLUDE_FLASH )
};
static const struct uvos_fs_driver_t *fs_driver = &uw_fs_driver;

/* Initialize logical file system, mounts and retrieves volume info
   Returns 0 on success, or fs_error_t (negative) on failure */
int UW_fs_init( void )
{
	uvos_fs_vol_info_t vol_info;

	/* Mount file system */
	if ( fs_driver->mount_fs() < 0 ) {
		return FS_ERR_MOUNT_FAILED;
	}

	/* Retrieve file system type */
	fres = fs_driver->get_vol_info( &vol_info );
	if ( fres ) {
		return FS_ERR_NO_VOL_INFO;
	}
	UW_fs_type = vol_info.type;

	return FS_ERR_OK;
}

/* Unmounts logical file system
   Returns FS_ERR_OK (0) on success, or FS_ERR_FAILED (negative) on failure */
int UW_fs_deinit( void )
{
	/* Unmount file system */
	if ( fs_driver->unmount_fs() < 0 ) {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}

/* Check that file system is mounted and of valid type */
bool UW_fs_is_valid( void )
{
	if ( ( !fs_driver->is_mounted() ) || ( UW_fs_type == FS_TYPE_INVALID ) ) {
		return false;
	}
	return true;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

// Get volume info (total and free Kbytes)
// Returns 0 on success, -1 if FS not valid, -2 on request failure
int UW_fs_get_vol_info( UW_fs_vol_info_t *UW_vol_info )
{
	uvos_fs_vol_info_t vol_info;

	/* File system should be mounted */
	if ( !fs_driver->is_mounted() ) {
		return FS_ERR_NOT_VALID;
	}

	fres = fs_driver->get_vol_info( &vol_info );
	if ( fres ) {
		return -2;
	}
	UW_vol_info->vol_total_Kbytes = vol_info.vol_total_Kbytes;
	UW_vol_info->vol_free_Kbytes = vol_info.vol_free_Kbytes;
	UW_vol_info->type = vol_info.type;

	return 0;
}

int UW_fs_read_file( const char *srcPath, uint8_t *buf, size_t bufSize )
{
	uvos_fs_file_t fsrc;
	uvos_fs_file_t *fp = &fsrc;
	int result;
	uint32_t bytes_read;

	/* open source file */
	result = fs_driver->file_open( fp, srcPath, FOPEN_MODE_R );
	if ( result < 0 ) {
		return FS_ERR_FAILED;
	}

	/* Read specified number of bytes into supplied buffer from opened file */
	result = fs_driver->file_read( fp, buf, bufSize, &bytes_read );
	if ( ( result < 0 ) || ( bytes_read != bufSize ) ) {
		return FS_ERR_FAILED;
	}

	/* close source file */
	result = fs_driver->file_close( fp );
	if ( result < 0 ) {
		return FS_ERR_FAILED;
	}
	return FS_ERR_OK;
}

int UW_fs_write_file( const char *filePath , const uint8_t *buf, size_t bufSize )
{
	uvos_fs_file_t file;
	uvos_fs_file_t *fp = &file;
	int result;
	uint32_t bytes_written;

	/* Open file for writing, create if necessary */
	result = fs_driver->file_open( fp, filePath, FOPEN_MODE_W );
	if ( result < 0 ) {
		return FS_ERR_FAILED;
	}

	/* Write supplied buffer to opened file */
	result = fs_driver->file_write( fp, buf, bufSize, &bytes_written );
	if ( result < 0 ) {
		fs_driver->file_close( fp );
		return FS_ERR_FAILED;
	}

	/* Clean up and exit */
	fs_driver->file_close( fp );
	return FS_ERR_OK;
}

// Open a file in a given mode per uvos_fopen_mode_t
// Returns 0 if file open is successful, -1 if unsuccessful
int UW_fs_file_open( UW_fs_file_t *file, const char *path, uvos_fopen_mode_t mode )
{

	/* Verify file system is mounted and valid type */
	if ( !UW_fs_is_valid() ) {
		return FS_ERR_NOT_VALID;
	}

	/* Assign mounted FS tpe to file handle passed to us */
	file->type = UW_fs_type;

	fres = fs_driver->file_open( file, path, mode );
	if ( fres ) {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}

// Read data from a file into a buffer
// Returns the number of bytes read, or -1 on failure
int UW_fs_file_read( UW_fs_file_t *file, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read )
{
	/* Verify file system is mounted and valid type */
	if ( !UW_fs_is_valid() ) {
		return FS_ERR_NOT_VALID;
	}

	fres = fs_driver->file_read( file, buf, bytes_to_read, bytes_read );
	if ( fres ) {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}

// Write data from a buffer into a file
// Returns the number of bytes written, or -1 on failure
int UW_fs_file_write( UW_fs_file_t *file, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written )
{
	/* Verify file system is mounted and valid type */
	if ( !UW_fs_is_valid() ) {
		return FS_ERR_NOT_VALID;
	}

	fres = fs_driver->file_write( file, buf, bytes_to_write, bytes_written );
	if ( fres ) {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}

// Seek to a position in a file
// Returns 0 on success, or -1 on failure
int UW_fs_file_seek( UW_fs_file_t *file, int32_t offset )
{
	if ( offset < 0 ) {
		return FS_ERR_RANGE;
	}

	/* Verify file system is mounted and valid type */
	if ( !UW_fs_is_valid() ) {
		return FS_ERR_NOT_VALID;
	}

	fres = fs_driver->file_seek( file, offset );
	if ( fres ) {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}

// Get the current position in a file
// Returns current read/write pointer of the file
uint32_t UW_fs_file_tell( UW_fs_file_t *file )
{
	return fs_driver->file_tell( file );
}

// Close a file
// Returns 0 on success, -1 if FS not valid, -2 on request failure
int UW_fs_file_close( UW_fs_file_t *file )
{
	/* Verify file system is mounted and valid type */
	if ( !UW_fs_is_valid() ) {
		return FS_ERR_NOT_VALID;
	}

	fres = fs_driver->file_close( file );
	if ( fres ) {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}

// Delete a file
// Returns 0 on success, -1 if FS not valid, -2 on request failure
int UW_fs_file_remove( const char *path )
{
	/* Verify file system is mounted and valid type */
	if ( !UW_fs_is_valid() ) {
		return FS_ERR_NOT_VALID;
	}

	fres = fs_driver->file_remove( path );
	if ( fres ) {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}

// Open a directory
// Returns 0 on success, -1 if FS not valid, -2 on request failure
int UW_fs_dir_open( UW_fs_dir_t *dir, const char *path )
{
	/* Verify file system is mounted and valid type */
	if ( !UW_fs_is_valid() ) {
		return FS_ERR_NOT_VALID;
	}

	fres = fs_driver->dir_open( ( uintptr_t * )dir, path );
	if ( fres ) {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}

// Close a directory
// Returns 0 on success, or -1 on failure
int UW_fs_dir_close( UW_fs_dir_t *dir )
{
	/* Verify file system is mounted and valid type */
	if ( !UW_fs_is_valid() ) {
		return FS_ERR_NOT_VALID;
	}

	fres = fs_driver->dir_close( ( uintptr_t * )dir );
	if ( fres ) {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}

// Read an entry in the dir info structure, based on the specified file or directory.
// Returns a positive value on success, 0 at the end of directory, or a negative error code on failure.
int UW_fs_dir_read( UW_fs_dir_t *dir, UW_fs_file_info_t *dir_info )
{
	/* Verify file system is mounted and valid type */
	if ( !UW_fs_is_valid() ) {
		return FS_ERR_NOT_VALID;
	}

	/* fres = positive value on success, 0 at the end of directory, or a negative error code on failure. */
	fres = fs_driver->dir_read( ( uintptr_t * )dir, ( uvos_file_info_t * )dir_info );
	if ( fres == 0 ) {
		return 0; // end of directory
	} else if ( fres > 0 ) {
		return true; // success
	} else {
		return FS_ERR_FAILED;
	}

	return FS_ERR_OK;
}