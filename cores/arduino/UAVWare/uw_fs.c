// #include <UAVWare.h>
#include <uvos.h>
#include "uw_fs.h"

#define PRN_BUFFER_SIZE  128

static uvos_fs_type_t UW_fs_type = FS_TYPE_INVALID;

/* Initialize logical file system, mounts and retrieves volume info
   Returns 0 on success, or fs_error_t (negative) on failure */
int UW_fs_init( void )
{
  struct uvos_fs_vol_info vol_info;
  int result;

  /* A valid file system should have been mounted in variant UVOS_Board_Init() */
  if ( !UVOS_FS_IsValid() ) {
    return FS_ERR_NOT_VALID;
  }

  /* Retrieve file system type */
  result = UVOS_FS_GetVolumeInfo( &vol_info );
  if ( result ) {
    return FS_ERR_NO_VOL_INFO;
  }
  UW_fs_type = vol_info.type;

  return FS_ERR_OK;
}

/* Check that file system is mounted and of valid type */
bool UW_fs_is_valid( void )
{
  if ( !UVOS_FS_IsValid() ) {
    return false;
  }
  return true;
}

// Get volume info (total and free Kbytes)
// Returns 0 on success, -1 if FS not valid, -2 on request failure
int UW_fs_get_vol_info( UW_fs_vol_info_t *UW_vol_info )
{
  struct uvos_fs_vol_info vol_info;
  int result;

  if ( !UVOS_FS_IsValid() ) {
    return FS_ERR_NOT_VALID;
  }

  result = UVOS_FS_GetVolumeInfo( &vol_info );
  if ( result ) {
    return FS_ERR_FAILED;
  }
  UW_vol_info->vol_total_Kbytes = vol_info.vol_total_Kbytes;
  UW_vol_info->vol_free_Kbytes = vol_info.vol_free_Kbytes;
  UW_vol_info->type = vol_info.type;

  return FS_ERR_OK;
}

// Find file size if file is not a directory
// Returns file size, or negative error code on failure
int UW_fs_get_file_size( const char *filePath )
{
  int32_t file_size;

  if ( !UVOS_FS_IsValid() ) {
    return FS_ERR_NOT_VALID;
  }

  file_size = UVOS_FS_FileSize( filePath );
  if ( file_size < 0 ) {
    return FS_ERR_FAILED;
  }

  return file_size;
}

int UW_fs_read_file( const char *srcPath, uint8_t *buf, size_t bufSize )
{
  struct uvos_fs_file fsrc;
  struct uvos_fs_file *fp = &fsrc;
  int result;
  uint32_t bytes_read;

  if ( !UVOS_FS_IsValid() ) {
    return FS_ERR_NOT_VALID;
  }

  /* open source file */
  result = UVOS_FS_FileOpen( fp, srcPath, FOPEN_MODE_R );
  if ( result < 0 ) {
    return FS_ERR_FAILED;
  }

  /* Read specified number of bytes into supplied buffer from opened file */
  result = UVOS_FS_FileRead( fp, buf, bufSize, &bytes_read );
  if ( ( result < 0 ) || ( bytes_read != bufSize ) ) {
    return FS_ERR_FAILED;
  }

  /* close source file */
  result = UVOS_FS_FileClose( fp );
  if ( result < 0 ) {
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

int UW_fs_write_file( const char *filePath , const uint8_t *buf, size_t bufSize )
{
  struct uvos_fs_file file;
  struct uvos_fs_file *fp = &file;
  int result;
  uint32_t bytes_written;

  if ( !UVOS_FS_IsValid() ) {
    return FS_ERR_NOT_VALID;
  }

  /* Open file for writing, create if necessary */
  result = UVOS_FS_FileOpen( fp, filePath, FOPEN_MODE_W );
  if ( result < 0 ) {
    return FS_ERR_FAILED;
  }

  /* Write supplied buffer to opened file */
  result = UVOS_FS_FileWrite( fp, buf, bufSize, &bytes_written );
  if ( result < 0 ) {
    UVOS_FS_FileClose( fp );
    return FS_ERR_FAILED;
  }

  /* Clean up and exit */
  UVOS_FS_FileClose( fp );
  return FS_ERR_OK;
}

int UW_fs_remove( const char *path )
{
  int result;

  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return FS_ERR_NOT_VALID;
  }

  result = UVOS_FS_Remove( path );
  if ( result ) {
    return FS_ERR_FAILED;
  }

  return FS_ERR_OK;
}

// Open a directory
// Returns 0 on success, -1 if FS not valid, -2 on request failure
int UW_fs_dir_open( UW_fs_dir_t *dir, const char *path )
{
  int result;

  /* Verify file system is mounted and valid type */
  if ( !UW_fs_is_valid() ) {
    return FS_ERR_NOT_VALID;
  }

  result = UVOS_FS_DirOpen( dir, path );
  if ( result ) {
    return FS_ERR_FAILED;
  }

  return FS_ERR_OK;
}

// Close a directory
// Returns FS_ERR_OK on success, or error code on failure
int UW_fs_dir_close( UW_fs_dir_t *dir )
{
  int result;

  /* Verify file system is mounted and valid type */
  if ( !UW_fs_is_valid() ) {
    return FS_ERR_NOT_VALID;
  }

  result = UVOS_FS_DirClose( dir );
  if ( result ) {
    return FS_ERR_FAILED;
  }

  return FS_ERR_OK;
}

// Read an entry in the dir info structure, based on the specified file or directory.
// Returns a positive value on success, 0 at the end of directory, or a negative error code on failure.
int UW_fs_dir_read( UW_fs_dir_t *dir, UW_fs_file_info_t *dir_info )
{
  int result;

  /* Verify file system is mounted and valid type */
  if ( !UW_fs_is_valid() ) {
    return FS_ERR_NOT_VALID;
  }

  /* result = positive value on success, 0 at the end of directory, or a negative error code on failure. */
  result = UVOS_FS_DirRead( dir, ( struct uvos_file_info * )dir_info );
  if ( result == 0 ) {
    return 0; // end of directory
  } else if ( result > 0 ) {
    return 1; // success
  } else {
    return FS_ERR_FAILED;
  }

  return FS_ERR_FAILED;
}

// Create a directory
// Returns 0 on success, or negative error code on failure
int UW_fs_mkdir( const char *path )
{
  int result;

  /* Verify file system is mounted and valid type */
  if ( !UW_fs_is_valid() ) {
    return FS_ERR_NOT_VALID;
  }

  result = UVOS_FS_Mkdir( path );
  if ( result ) {
    return FS_ERR_FAILED;
  }

  return FS_ERR_OK;
}

#if 0 // gls

int UW_fs_dir( const char *path )
{
  int result;
  UW_fs_dir_t dir;
  UW_fs_file_info_t info;

  char prn_buffer[ PRN_BUFFER_SIZE ] = {0};

  /* Verify file system is mounted and valid type */
  if ( !UW_fs_is_valid() ) {
    return FS_ERR_NOT_VALID;
  }

  if ( path == NULL ) {
    path = "/"; /* default path */
  }

  result = UW_fs_dir_open( &dir, path );
  if ( result ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "FAILED UW_fs_dir_open()!\r\n" );
    return FS_ERR_FAILED;
  }

  for ( ;; ) {
    // Read an entry in the dir info structure, based on the specified file or directory.
    // Returns a positive value on success, 0 at the end of directory, or a negative error code on failure.
    result = UW_fs_dir_read( &dir, &info );
    if ( result < 0 ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "FAILED UW_fs_dir_read()!\r\n" );
      return FS_ERR_FAILED;
    }

    if ( result == 0 ) { /* no more files */
      break;
    }

    if ( info.is_dir ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "dir " );
    }  else {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "reg " );
    }

    static const char *prefixes[] = {"", "K", "M", "G"};  /* prefixes for kilo, mega and giga */
    for ( int i = sizeof( prefixes ) / sizeof( prefixes[0] ) - 1; i >= 0; i-- ) {
      if ( info.size >= ( 1 << 10 * i ) - 1 ) {
        uint16_t prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%*u%sB ", 4 - ( i != 0 ), ( unsigned int )info.size >> 10 * i, prefixes[i] );
        UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
        break;
      }
    }
    UVOS_COM_SendString( UVOS_COM_DEBUG, info.name );
    UVOS_COM_SendString( UVOS_COM_DEBUG, "\r\n" );
  }

  result = UW_fs_dir_close( &dir );
  if ( result ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "FAILED lfs_dir_close()!\r\n" );
    return FS_ERR_FAILED;
  }

  return FS_ERR_OK;
}

#endif // gls