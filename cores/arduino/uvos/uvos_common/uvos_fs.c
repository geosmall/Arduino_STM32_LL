#include <uvos.h>
#include <uvos_fs_priv.h>

#ifdef UVOS_INCLUDE_FS

static const struct uvos_fs_driver *uvos_fs_driver;

/* Define an abstract file system object compatible with both FatFS and LittleFS */
// struct uvos_fs {
//   // Union of FatFS and lfs_t must be first member of struct
//   union {
//     FATFS FatFS; // FatFS file system object
//     lfs_t LittleFS; // LittleFS file system object
//   };
//   uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
// };
// static struct uvos_fs uv_fs;

static struct uvos_fs_vol_info vol_info;
static int fres;

/**
 * Initialises file system
 * \param[in] pointer to file system driver interface to be used
 * \return 0 if file open is successful, negative number if unsuccessful
 */
int32_t UVOS_FS_Init( const struct uvos_fs_driver *fs_driver )
{
  vol_info.type = FS_TYPE_INVALID;

  /* Hook up fthe driver to be used to access the file system */
  uvos_fs_driver = fs_driver;

  /* Mount file system */
  if ( uvos_fs_driver->mount_fs() < 0 ) {
    return -1;
  }

  /* Retrieve file system volume information, including it's type */
  fres = uvos_fs_driver->get_vol_info( &vol_info );
  if ( fres ) {
    return -2;
  }

  return 0;
}

/* Check that file system is mounted and of valid type */
bool UVOS_FS_IsValid( void )
{
  if ( ( !uvos_fs_driver->is_mounted() ) || ( vol_info.type == FS_TYPE_INVALID ) ) {
    return false;
  }
  return true;
}

int32_t UVOS_FS_GetVolumeInfo( struct uvos_fs_vol_info *info )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  /* Update file system volume info */
  fres = uvos_fs_driver->get_vol_info( &vol_info );
  if ( fres ) {
    return -2;
  }

  /* Copy local vol_info result into passed in vol info struct */
  *info = vol_info;

  return 0;
}

/* Unmounts logical file system
   Returns 0 if file open is successful, negative number if unsuccessful */
int32_t UVOS_FS_Deinit( void )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  /* Unmount file system */
  if ( uvos_fs_driver->unmount_fs() < 0 ) {
    return -2;
  }

  return 0;
}

/* Open a file in a given mode per uvos_fopen_mode_t
   Returns 0 if file open is successful, negative number if unsuccessful */
int32_t UVOS_FS_file_open( struct uvos_fs_file *file, const char *path, uvos_fopen_mode_t mode )
{

  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  /* Assign mounted FS tpe to file handle passed to us */
  file->type = vol_info.type;

  fres = uvos_fs_driver->file_open( file, path, mode );
  if ( fres ) {
    return -2;
  }

  return 0;
}

/* Read data from a file into a buffer
   Returns 0 if file open is successful, negative number if unsuccessful */
int32_t UVOS_FS_FileRead( struct uvos_fs_file *file, void *buf, uint32_t bytes_to_read, uint32_t *bytes_read )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  fres = uvos_fs_driver->file_read( file, buf, bytes_to_read, bytes_read );
  if ( fres ) {
    return -2;
  }

  return 0;
}

/* Write data from a buffer into a file
   Returns 0 if file open is successful, negative number if unsuccessful */
int32_t UVOS_FS_FileWrite( struct uvos_fs_file *file, const void *buf, uint32_t bytes_to_write, uint32_t *bytes_written )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  fres = uvos_fs_driver->file_write( file, buf, bytes_to_write, bytes_written );
  if ( fres ) {
    return -2;
  }

  return 0;
}

/* Seek to a position in a file
   Returns 0 if file open is successful, negative number if unsuccessful */
int32_t UVOS_FS_FileSeek( struct uvos_fs_file *file, int32_t offset )
{
  if ( offset < 0 ) { offset = 0; }

  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  fres = uvos_fs_driver->file_seek( file, offset );
  if ( fres ) {
    return -2;
  }

  return 0;
}


/* Get the current position in a file
   Returns current read/write pointer of the file */
uint32_t UVOS_FS_FileTell( struct uvos_fs_file *file )
{
  return uvos_fs_driver->file_tell( file );
}

/* Close a file
   Returns 0 if file open is successful, negative number if unsuccessful */
int32_t UVOS_FS_FileClose( struct uvos_fs_file *file )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  fres = uvos_fs_driver->file_close( file );
  if ( fres ) {
    return -2;
  }

  return 0;
}

/* Delete a file or directory
   Returns 0 if file open is successful, negative number if unsuccessful */
int32_t UVOS_FS_Remove( const char *path )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  fres = uvos_fs_driver->remove( path );
  if ( fres ) {
    return -2;
  }

  return 0;
}

/* Open a directory
   Returns 0 if file open is successful, negative number if unsuccessful */
int32_t UVOS_FS_DirOpen( struct uvos_fs_dir *dir, const char *path )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  fres = uvos_fs_driver->dir_open( dir, path );
  if ( fres ) {
    return -2;
  }

  return 0;
}

/* Close a directory
   Returns 0 if file open is successful, negative number if unsuccessful */
int32_t UVOS_FS_DirClose( struct uvos_fs_dir *dir )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  fres = uvos_fs_driver->dir_close( dir );
  if ( fres ) {
    return -2;
  }

  return 0;
}

/* Read an entry in the dir info structure, based on the specified file or directory.
   Returns a positive value on success, 0 at the end of directory, or a negative error code on failure. */
int32_t UVOS_FS_DirRead( struct uvos_fs_dir *dir, struct uvos_file_info *dir_info )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  /* dir_read() returns positive value on success, 0 at the end of directory, or a negative error code on failure. */
  fres = uvos_fs_driver->dir_read( dir, ( struct uvos_file_info * )dir_info );
  if ( fres == 0 ) {
    return 0; // end of directory
  } else if ( fres > 0 ) {
    return true; // success
  } else {
    return -2;
  }

  return -2;
}

int32_t UVOS_FS_Mkdir( const char *path )
{
  /* Verify file system is mounted and valid type */
  if ( !UVOS_FS_IsValid() ) {
    return -1;
  }

  fres = uvos_fs_driver->mkdir( path );
  if ( fres ) {
    return -2;
  }

  return 0;
}


#endif // UVOS_INCLUDE_FS