#include <uvos.h>
#include <uvos_fs_priv.h>

#ifdef UVOS_INCLUDE_FS

static const struct uvos_fs_driver *uvos_fs_driver;

// Define file system object compatible with both FatFS and LittleFS
typedef struct {
  // Union of FatFS and lfs_t must be first member of struct
  union {
    FATFS FatFS; // FatFS file system object
    lfs_t LittleFS; // LittleFS file system object
  };
  uvos_fs_type_t type; // File system type (invalid, FatFS, LittleFS)
} uvos_fs_t;

/**
 * Initialises file system
 * \param[in] pointer to file system driver interface to be used
 * \return < 0 if initialisation failed
 */
// int32_t UVOS_FS_Init( uintptr_t *fs_driver )
int32_t UVOS_FS_Init( const struct uvos_fs_driver *fs_driver )
{
	uvos_fs_driver = fs_driver;

	return 0;
}

int32_t UVOS_FS_Mount( void )
{
		/* Mount file system */
	if ( uvos_fs_driver->mount_fs() < 0 ) {
		return -1;
	}

	return 0;
}


#endif // UVOS_INCLUDE_LED