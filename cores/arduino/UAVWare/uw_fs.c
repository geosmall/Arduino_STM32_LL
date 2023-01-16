#include <UAVWare.h>
#include <uvos.h>
#include <lfs.h>
#include "uw_fs.h"
#include <uvos_flash_jedec_priv.h>
#include "UTIL1.h"

#define FS_FILE_NAME_SIZE  32 /* Length of file name, used in FS buffers */
#define LFS_CACHE_SIZE 256
#define LFS_LOOKAHEAD_SIZE 512
#define PRN_BUFFER_SIZE  128

static int block_device_read( const struct lfs_config * c, lfs_block_t block,  lfs_off_t off, void * buffer, lfs_size_t size );
static int block_device_prog( const struct lfs_config * c, lfs_block_t block, lfs_off_t off, const void * buffer, lfs_size_t size );
static int block_device_erase( const struct lfs_config * c, lfs_block_t block );
static int block_device_sync( const struct lfs_config * c );

// Optional statically allocated read buffer. Must be cache_size.
// By default lfs_malloc is used to allocate this buffer.
uint8_t read_buf[ LFS_CACHE_SIZE ];

// Optional statically allocated program buffer. Must be cache_size.
// By default lfs_malloc is used to allocate this buffer.
uint8_t prog_buf[ LFS_CACHE_SIZE ];

// Optional statically allocated lookahead buffer. Must be lookahead_size
// and aligned to a 32-bit boundary. By default lfs_malloc is used to
// allocate this buffer.
uint8_t lookahead_buf[ LFS_LOOKAHEAD_SIZE ] __attribute__ ( ( aligned ( 4 ) ) );

// configuration of the file system is provided by this struct
const struct lfs_config FS_cfg = {
  // block device operations
  .read  = block_device_read,
  .prog  = block_device_prog,
  .erase = block_device_erase,
  .sync  = block_device_sync,
  // chip info
  .read_size = LFS_CACHE_SIZE,
  .prog_size = LFS_CACHE_SIZE,
  .block_size = 4096,
  .block_count = 4096,
  .block_cycles = 500,
  .cache_size = LFS_CACHE_SIZE,
  .lookahead_size = LFS_LOOKAHEAD_SIZE,
  .read_buffer = read_buf,
  .prog_buffer = prog_buf,
  .lookahead_buffer = lookahead_buf,
};

/* variables used by the file system */
static bool _FS_isMounted = false;
static lfs_t _FS_lfs;
// static uintptr_t _flash_id;

static int block_device_read( const struct lfs_config * c, lfs_block_t block,  lfs_off_t off, void * buffer, lfs_size_t size )
{
  uint32_t result;

  // result = UVOS_Flash_Jedec_ReadData( _flash_id, block * c->block_size + off, buffer, size );
  result = UVOS_Flash_Jedec_ReadData( UVOS_FLASH_SPI_PORT, block * c->block_size + off, buffer, size );
  if ( result ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_prog( const struct lfs_config * c, lfs_block_t block, lfs_off_t off, const void * buffer, lfs_size_t size )
{
  uint32_t result;

  // result = UVOS_Flash_Jedec_WriteData( _flash_id, block * c->block_size + off, ( uint8_t * )buffer, size );
  result = UVOS_Flash_Jedec_WriteData( UVOS_FLASH_SPI_PORT, block * c->block_size + off, ( uint8_t * )buffer, size );
  if ( result ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_erase( const struct lfs_config * c, lfs_block_t block )
{
  uint32_t result;

  // result = UVOS_Flash_Jedec_EraseSector( _flash_id, block * c->block_size );
  result = UVOS_Flash_Jedec_EraseSector( UVOS_FLASH_SPI_PORT, block * c->block_size );
  if ( result ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_sync( const struct lfs_config * c )
{
  return LFS_ERR_OK;
}

int UW_fs_init( void )
{
  /* Save local pointer to SPI flash */
  // _flash_id = UVOS_FLASH_SPI_PORT;

  // UVOS_COM_SendString( UVOS_COM_DEBUG, "Attempting to mount existing media\r\n" );
  if ( UW_fs_mount() != FS_ERR_OK ) {
    /* FS mount failed, try formatting underlying FS */
    UVOS_COM_SendString( UVOS_COM_DEBUG, "Could not mount media, attemping to format\r\n" );
    if ( UW_fs_format() != FS_ERR_OK ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "Format failed\r\n" );
      return FS_ERR_FAILED;
    }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "Attempting to mount freshly formatted media\r\n" );
    if ( UW_fs_mount() != FS_ERR_OK ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "Mount after format failed\r\n" );
      return FS_ERR_FAILED;
    }
  }

  return FS_ERR_OK;
}

int UW_fs_read_file( const char * srcPath, uint8_t * buf, size_t bufSize )
{
  lfs_file_t fsrc;
  int result;

#ifdef LFS_NO_MALLOC
  static uint8_t file_buffer[ LFS_CACHE_SIZE ];
  static struct lfs_file_config file_cfg = {
    .buffer = file_buffer
  };
#endif

  /* open source file */
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &fsrc, srcPath, LFS_O_RDONLY, &file_cfg );
#else
  result = lfs_file_open( &_FS_lfs, &fsrc, srcPath, LFS_O_RDONLY );
#endif
  if ( result < 0 ) {
    return FS_ERR_FAILED;
  }

  result = lfs_file_read( &_FS_lfs, &fsrc, buf, bufSize );
  if ( result != bufSize ) {
    return FS_ERR_FAILED;
  }

  /* close source file */
  result = lfs_file_close( &_FS_lfs, &fsrc );
  if ( result < 0 ) {
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

int UW_fs_write_file( const char * filePath , const uint8_t * buf, size_t bufSize )
{
  lfs_file_t file;
  int result;

#ifdef LFS_NO_MALLOC
  static uint8_t file_buffer[ LFS_CACHE_SIZE ];
  static struct lfs_file_config file_cfg = {
    .buffer = file_buffer
  };
#endif

  if ( !_FS_isMounted ) {
    return FS_ERR_FAILED;
  }
  /* Delete existing file */
  UW_fs_remove_file( filePath );

  /* Open file for writing, create if necessary */
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &file, filePath, LFS_O_WRONLY | LFS_O_CREAT, &file_cfg );
#else
  result = lfs_file_open( &_FS_lfs, &file, filePath, LFS_O_WRONLY | LFS_O_CREAT );
#endif
  if ( result < 0 ) {
    return FS_ERR_FAILED;
  }

  /* Write supplied buffer to opened file */
  if ( lfs_file_write( &_FS_lfs, &file, buf, bufSize ) < 0 ) {
    lfs_file_close( &_FS_lfs, &file );
    return FS_ERR_FAILED;
  }

  /* Clean up and exit */
  lfs_file_close( &_FS_lfs, &file );
  return FS_ERR_FAILED;
}

int UW_fs_format( void )
{
  int result;

  if ( _FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "File system is already mounted, unmount it first.\r\n" );
    return FS_ERR_FAILED;
  }

  UVOS_COM_SendString( UVOS_COM_DEBUG, "Formatting ..." );

  result = lfs_format( &_FS_lfs, &FS_cfg );
  if ( result == LFS_ERR_OK ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, " done.\r\n" );
    return FS_ERR_OK;
  } else {
    UVOS_COM_SendString( UVOS_COM_DEBUG, " FAILED!\r\n" );
    return FS_ERR_FAILED;
  }
}

int UW_fs_mount( void )
{
  int result;

  if ( _FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "File system is already mounted, unmount it first.\r\n" );
    return FS_ERR_FAILED;
  }

  UVOS_COM_SendString( UVOS_COM_DEBUG, "Mounting ..." );

  result = lfs_mount( &_FS_lfs, &FS_cfg );

  if ( result == LFS_ERR_OK ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, " done.\r\n" );
    _FS_isMounted = true;
    return FS_ERR_OK;
  } else {
    UVOS_COM_SendString( UVOS_COM_DEBUG, " FAILED!\r\n" );
    return FS_ERR_FAILED;
  }
}

int UW_fs_unmount( void )
{
  int result;

  if ( !_FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "File system is already unmounted.\r\n" );
    return FS_ERR_FAILED;
  }
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Unmounting ..." );
  result = lfs_unmount( &_FS_lfs );
  if ( result == LFS_ERR_OK ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, " done.\r\n" );
    _FS_isMounted = false;
    return FS_ERR_OK;
  } else {
    UVOS_COM_SendString( UVOS_COM_DEBUG, " FAILED!\r\n" );
    return FS_ERR_FAILED;
  }
}

int UW_fs_dir( const char * path )
{
  int result;
  lfs_dir_t dir;
  struct lfs_info info;

  char prn_buffer[ PRN_BUFFER_SIZE ] = {0};

  if ( !_FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "File system is not mounted, mount it first.\r\n" );
    return FS_ERR_FAILED;
  }
  if ( path == NULL ) {
    path = "/"; /* default path */
  }
  result = lfs_dir_open( &_FS_lfs, &dir, path );
  if ( result != LFS_ERR_OK ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "FAILED lfs_dir_open()!\r\n" );
    return FS_ERR_FAILED;
  }
  for ( ;; ) {
    result = lfs_dir_read( &_FS_lfs, &dir, &info );
    if ( result < 0 ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "FAILED lfs_dir_read()!\r\n" );
      return FS_ERR_FAILED;
    }
    if ( result == 0 ) { /* no more files */
      break;
    }
    switch ( info.type ) {
    case LFS_TYPE_REG: UVOS_COM_SendString( UVOS_COM_DEBUG, "reg " ); break;
    case LFS_TYPE_DIR: UVOS_COM_SendString( UVOS_COM_DEBUG, "dir " ); break;
    default:           UVOS_COM_SendString( UVOS_COM_DEBUG, "?   " ); break;
    }
    static const char * prefixes[] = {"", "K", "M", "G"}; /* prefixes for kilo, mega and giga */
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

  result = lfs_dir_close( &_FS_lfs, &dir );
  if ( result != LFS_ERR_OK ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "FAILED lfs_dir_close()!\r\n" );
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

int UW_fs_copy_file( const char * srcPath, const char * dstPath )
{
  lfs_file_t fsrc, fdst;
  __attribute__( ( unused ) ) volatile uint8_t res;
  int result, nofBytesRead;
  uint8_t buffer[32];   /* copy buffer */

#ifdef LFS_NO_MALLOC
  static uint8_t                file_buffer_src[LFS_CACHE_SIZE];
  static struct lfs_file_config file_cfg_src = {
    .buffer = file_buffer_src
  };
  static uint8_t                file_buffer_dst[LFS_CACHE_SIZE];
  static struct lfs_file_config file_cfg_dst = {
    .buffer = file_buffer_dst
  };
#endif

  if ( !_FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }

  /* open source file */
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &fsrc, srcPath, LFS_O_RDONLY, &file_cfg_src );
#else
  result = lfs_file_open( &_FS_lfs, &fsrc, srcPath, LFS_O_RDONLY );
#endif
  if ( result < 0 ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed opening source file!\r\n" );
    return FS_ERR_FAILED;
  }
  /* create destination file */
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &fdst, dstPath, LFS_O_WRONLY | LFS_O_CREAT, &file_cfg_dst );
#else
  result = lfs_file_open( &_FS_lfs, &fdst, dstPath, LFS_O_WRONLY | LFS_O_CREAT );
#endif
  if ( result < 0 ) {
    lfs_file_close( &_FS_lfs, &fsrc );
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed opening destination file!\r\n" );
    return FS_ERR_FAILED;
  }
  /* now copy source to destination */
  for ( ;; ) {
    nofBytesRead = lfs_file_read( &_FS_lfs, &fsrc, buffer, sizeof( buffer ) );
    if ( nofBytesRead < 0 ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed reading source file!\r\n" );
      res = FS_ERR_FAILED;
      break;
    }
    if ( nofBytesRead == 0 ) { /* end of file */
      break;
    }
    result = lfs_file_write( &_FS_lfs, &fdst, buffer, nofBytesRead );
    if ( result < 0 ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed writing destination file!\r\n" );
      res = FS_ERR_FAILED;
      break;
    }
  }
  /* close all files */
  result = lfs_file_close( &_FS_lfs, &fsrc );
  if ( result < 0 ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed closing source file!\r\n" );
    res = FS_ERR_FAILED;
  }
  result = lfs_file_close( &_FS_lfs, &fdst );
  if ( result < 0 ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed closing destination file!\r\n" );
    res = FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

int UW_fs_move_file( const char * srcPath, const char * dstPath )
{
  if ( !_FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }
  if ( lfs_rename( &_FS_lfs, srcPath, dstPath ) < 0 ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: failed renaming file or directory.\r\n" );
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

// Find file size if file is not a directory
// Returns file size, or negative error code on failure
int UW_fs_file_size( const char * filePath )
{
  struct lfs_info info;
  int result;

  if ( !_FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }
  result = lfs_stat( &_FS_lfs, filePath, &info );
  if ( result == 0 ) {
    if ( info.type == LFS_TYPE_REG ) {
      return info.size;
    } else {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File is a directory.\r\n" );
      return FS_ERR_FAILED;
    }
  }
  return FS_ERR_FAILED;
}

int UW_fs_remove_file( const char * filePath )
{
  int result;

  if ( !_FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }
  result = lfs_remove( &_FS_lfs, filePath );
  if ( result < 0 ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: Failed removing file.\r\n" );
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

int UW_fs_run_benchmark( void )
{
  lfs_file_t file;
  int result;
  uint32_t i;
  uint8_t read_buf[10];
  unsigned int start_microseconds;
  unsigned int microseconds;
  uint16_t prn_buflen;
  char prn_buffer[ PRN_BUFFER_SIZE ] = {0};

#ifdef LFS_NO_MALLOC
  static uint8_t file_buffer[ LFS_CACHE_SIZE ];
  static struct lfs_file_config file_cfg = {
    .buffer = file_buffer
  };
#endif

  if ( !_FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Benchmark: write/copy/read a 100kB file:\r\n" );
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Delete existing benchmark files...\r\n" );
  UW_fs_remove_file( "./bench.txt" );
  UW_fs_remove_file( "./copy.txt" );

  UVOS_COM_SendString( UVOS_COM_DEBUG, "Create benchmark file...\r\n" );
  start_microseconds = micros();
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &file, "./bench.txt", LFS_O_WRONLY | LFS_O_CREAT, &file_cfg );
#else
  result = lfs_file_open( &_FS_lfs, &file, "./bench.txt", LFS_O_WRONLY | LFS_O_CREAT );
#endif
  if ( result < 0 ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed creating benchmark file!\r\n" );
    return FS_ERR_FAILED;
  }
  for ( i = 0; i < 10240; i++ ) {
    if ( lfs_file_write( &_FS_lfs, &file, "benchmark ", sizeof( "benchmark " ) - 1 ) < 0 ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed writing file!\r\n" );
      lfs_file_close( &_FS_lfs, &file );
      return FS_ERR_FAILED;
    }
  }
  lfs_file_close( &_FS_lfs, &file );

  microseconds = micros() - start_microseconds;

  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u", microseconds / 1000 );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  UVOS_COM_SendString( UVOS_COM_DEBUG, " ms for writing (" );
  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u",  ( unsigned int )( 100.0f  / ( microseconds / 1000000.0f ) ) );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  UVOS_COM_SendString( UVOS_COM_DEBUG, " kB/s)\r\n" );

  /* read benchmark */
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Read 100kB benchmark file...\r\n" );

  start_microseconds = micros();

#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &file, "./bench.txt", LFS_O_RDONLY, &file_cfg );
#else
  result = lfs_file_open( &_FS_lfs, &file, "./bench.txt", LFS_O_RDONLY );
#endif
  if ( result < 0 ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed opening benchmark file!\r\n" );
    return FS_ERR_FAILED;
  }
  for ( i = 0; i < 10240; i++ ) {
    if ( lfs_file_read( &_FS_lfs, &file, &read_buf[0], sizeof( read_buf ) ) < 0 ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed reading file!\r\n" );
      lfs_file_close( &_FS_lfs, &file );
      return FS_ERR_FAILED;
    }
  }
  lfs_file_close( &_FS_lfs, &file );

  microseconds = micros() - start_microseconds;

  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u", microseconds / 1000 );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  UVOS_COM_SendString( UVOS_COM_DEBUG, " ms for reading (" );
  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u",  ( unsigned int )( 100.0f  / ( microseconds / 1000000.0f ) ) );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  UVOS_COM_SendString( UVOS_COM_DEBUG, " kB/s)\r\n" );

  /* copy benchmark */
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Copy 100kB file...\r\n" );

  start_microseconds = micros();

  UW_fs_copy_file( ( const char * )"./bench.txt", ( const char * )"./copy.txt" );

  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u", microseconds / 1000 );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  UVOS_COM_SendString( UVOS_COM_DEBUG, " ms for copy (" );
  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u",  ( unsigned int )( 100.0f  / ( microseconds / 1000000.0f ) ) );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  UVOS_COM_SendString( UVOS_COM_DEBUG, " kB/s)\r\n" );

  UVOS_COM_SendString( UVOS_COM_DEBUG, "done!\r\n" );
  return FS_ERR_OK;
}

// static uint8_t FS_PrintStatus(CLS1_ConstStdIOType *io) {
__attribute__( ( unused ) ) static uint32_t UW_fs_print_status( void )
{
  __attribute__( ( unused ) ) uint8_t buf[24];

  // CLS1_SendStatusStr((const unsigned char*)"FS", (const unsigned char*)"\r\n", io->stdOut);
  // CLS1_SendStatusStr((const unsigned char*)"  mounted", _FS_isMounted ? "yes\r\n" : "no\r\n", io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_count * FS_cfg.block_size);
  // UTIL1_strcat(buf, sizeof(buf), " bytes\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  space", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.read_size);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  read_size", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.prog_size);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  prog_size", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_size);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  block_size", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_count);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  block_count", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.lookahead);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  lookahead", buf, io->stdOut);
  return FS_ERR_OK;
}

#if 0 // GLS

static void UTIL1_strcpy( uint8_t * dst, size_t dstSize, const unsigned char * src )
{
  dstSize--; /* for zero byte */
  while ( dstSize > 0 && *src != '\0' ) {
    *dst++ = *src++;
    dstSize--;
  }
  *dst = '\0';
}

static void UTIL1_strcat( uint8_t * dst, size_t dstSize, const unsigned char * src )
{
  dstSize--; /* for zero byte */
  /* point to the end of the source */
  while ( dstSize > 0 && *dst != '\0' ) {
    dst++;
    dstSize--;
  }
  /* copy the src in the destination */
  while ( dstSize > 0 && *src != '\0' ) {
    *dst++ = *src++;
    dstSize--;
  }
  /* terminate the string */
  *dst = '\0';
}

static void UTIL1_strcatNum8Hex( uint8_t * dst, size_t dstSize, uint8_t num )
{
  unsigned char buf[sizeof( "FF" )]; /* maximum buffer size we need */
  unsigned char hex;

  buf[2] = '\0';
  hex = ( char )( num & 0x0F );
  buf[1] = ( char )( hex + ( ( hex <= 9 ) ? '0' : ( 'A' - 10 ) ) );
  hex = ( char )( ( num >> 4 ) & 0x0F );
  buf[0] = ( char )( hex + ( ( hex <= 9 ) ? '0' : ( 'A' - 10 ) ) );
  UTIL1_strcat( dst, dstSize, buf );
}

static void UTIL1_strcatNum16Hex( uint8_t * dst, size_t dstSize, uint16_t num )
{
  unsigned char buf[sizeof( "FFFF" )]; /* maximum buffer size we need */
  unsigned char hex;
  int8_t i;

  buf[4] = '\0';
  i = 3;
  do {
    hex = ( char )( num & 0x0F );
    buf[i] = ( char )( hex + ( ( hex <= 9 ) ? '0' : ( 'A' - 10 ) ) );
    num >>= 4;                          /* next nibble */
    i--;
  } while ( i >= 0 );
  UTIL1_strcat( dst, dstSize, buf );
}

static void UTIL1_strcatNum24Hex( uint8_t * dst, size_t dstSize, uint32_t num )
{
  unsigned char buf[sizeof( "FFFFFF" )]; /* maximum buffer size we need */
  unsigned char hex;
  int8_t i;

  buf[6] = '\0';
  i = 5;
  do {
    hex = ( char )( num & 0x0F );
    buf[i] = ( char )( hex + ( ( hex <= 9 ) ? '0' : ( 'A' - 10 ) ) );
    num >>= 4;                          /* next nibble */
    i--;
  } while ( i >= 0 );
  UTIL1_strcat( dst, dstSize, buf );
}

static void UTIL1_strcatNum32Hex( uint8_t * dst, size_t dstSize, uint32_t num )
{
  unsigned char buf[sizeof( "FFFFFFFF" )]; /* maximum buffer size we need */
  unsigned char hex;
  int8_t i;

  buf[8] = '\0';
  i = 7;
  do {
    hex = ( char )( num & 0x0F );
    buf[i] = ( char )( hex + ( ( hex <= 9 ) ? '0' : ( 'A' - 10 ) ) );
    num >>= 4;                          /* next nibble */
    i--;
  } while ( i >= 0 );
  UTIL1_strcat( dst, dstSize, buf );
}

static void UTIL1_strcatNumHex( uint8_t * dst, size_t dstSize, uint32_t num, uint8_t nofBytes )
{
  if ( nofBytes == 1 ) {
    UTIL1_strcatNum8Hex( dst, dstSize, ( uint8_t )num );
  } else if ( nofBytes == 2 ) {
    UTIL1_strcatNum16Hex( dst, dstSize, ( uint16_t )num );
  } else if ( nofBytes == 3 ) {
    UTIL1_strcatNum24Hex( dst, dstSize, num );
  } else { /* nofBytes==4 */
    UTIL1_strcatNum32Hex( dst, dstSize, num );
  }
}

static void UTIL1_chcat( uint8_t * dst, size_t dstSize, uint8_t ch )
{
  dstSize--; /* for zero byte */
  /* point to the end of the source */
  while ( dstSize > 0 && *dst != '\0' ) {
    dst++;
    dstSize--;
  }
  /* copy the ch in the destination */
  if ( dstSize > 0 ) {
    *dst++ = ch;
  }
  /* terminate the string */
  *dst = '\0';
}

#endif // GLS

static int printMemory( void * hndl, uint32_t startAddr, uint32_t endAddr, uint8_t addrSize, uint8_t bytesPerLine, uint8_t ( *readfp )( void *, uint32_t, uint8_t *, size_t ) )
{
#define NOF_BYTES_PER_LINE 32 /* how many bytes are shown on a line. This defines as well the chunk size we read from memory */
#define MAX_NOF_BYTES_PER_LINE 32
  uint8_t buf[ MAX_NOF_BYTES_PER_LINE ]; /* this is the chunk of data we get (per line in output) */
  uint8_t str[ 3 * MAX_NOF_BYTES_PER_LINE + ( ( MAX_NOF_BYTES_PER_LINE + 1 ) / 8 ) + 1 ]; /* maximum string for output:
                                              - '3*' because each byte is 2 hex digits plus a space
                                              - '(NOF_BYTES_PER_LINE+1)/8' because we add a space between every 8 byte block
                                              - '+1' for the final zero byte */
  uint32_t addr;
  uint8_t j;
  uint8_t bufSize;
  uint8_t ch;

  if ( endAddr < startAddr ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "\r\n*** End address must be larger or equal than start address\r\n" );
    return FS_ERR_RANGE;
  }
  for ( addr = startAddr; addr <= endAddr; /* nothing */ ) {
    if ( endAddr - addr + 1 >= bytesPerLine ) { /* read only part of buffer */
      bufSize = bytesPerLine; /* read full buffer */
    } else {
      bufSize = ( uint8_t )( endAddr - addr + 1 );
    }
    if ( readfp( hndl, addr, buf, bufSize ) != FS_ERR_OK ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "\r\n*** Read failed!\r\n" );
      return FS_ERR_FAILED;
    }
    /* write address */
    UTIL1_strcpy( str, sizeof( str ), ( unsigned char * )"0x" );
    UTIL1_strcatNumHex( str, sizeof( str ), addr, addrSize );
    UTIL1_chcat( str, sizeof( str ), ':' );
    UVOS_COM_SendString( UVOS_COM_DEBUG, ( char * )str );
    /* write data in hex */
    str[0] = '\0';
    for ( j = 0; j < bufSize; j++ ) {
      if ( ( j ) == 0 ) {
        UTIL1_chcat( str, sizeof( str ), ' ' );
      }
      UTIL1_strcatNum8Hex( str, sizeof( str ), buf[j] );
      UTIL1_chcat( str, sizeof( str ), ' ' );
    }
    for ( /*empty*/; j < bytesPerLine; j++ ) { /* fill up line */
      UTIL1_strcat( str, sizeof( str ), ( unsigned char * )"-- " );
    }
    UVOS_COM_SendString( UVOS_COM_DEBUG, ( char * )str );
    /* write in ASCII */
    UVOS_COM_SendString( UVOS_COM_DEBUG, " " );
    for ( j = 0; j < bufSize; j++ ) {
      ch = buf[j];
      if ( ch >= ' ' && ch <= 0x7f ) {
        UVOS_COM_SendChar( UVOS_COM_DEBUG, ch );
      } else {
        UVOS_COM_SendString( UVOS_COM_DEBUG, "." );
      }
    }
    for ( /*empty*/; j < bytesPerLine; j++ ) { /* fill up line */
      UTIL1_strcat( str, sizeof( str ), ( unsigned char * )"-- " );
    }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "\r\n" );
    addr += bytesPerLine;
  }
  return FS_ERR_OK;
}

static uint8_t readFromFile( void * hndl, uint32_t addr, uint8_t * buf, size_t bufSize )
{
  lfs_file_t * fp;

  fp = ( lfs_file_t * )hndl;
  if ( lfs_file_read( &_FS_lfs, fp, buf, bufSize ) < 0 ) {
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

int UW_fs_print_hex_file( const char * filePath )
{
  lfs_file_t file;
  uint8_t res = FS_ERR_OK;
  int32_t fileSize;
  int result;

#ifdef LFS_NO_MALLOC
  static uint8_t file_buffer[ LFS_CACHE_SIZE ];
  static struct lfs_file_config file_cfg = {
    .buffer = file_buffer
  };
#endif

  if ( !_FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &file, filePath, LFS_O_RDONLY, &file_cfg );
#else
  result = lfs_file_open( &_FS_lfs, &file, filePath, LFS_O_RDONLY );
#endif
  if ( result < 0 ) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: Failed opening file.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: Failed opening file.\r\n" );
    return FS_ERR_FAILED;
  }
  fileSize = lfs_file_size( &_FS_lfs, &file );
  if ( fileSize < 0 ) {
    // if (io != NULL) {
    //   CLS1_SendStr("ERROR: getting file size\r\n", io->stdErr);
    //   (void)lfs_file_close(&_FS_lfs, &file);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: getting file size\r\n" );
    lfs_file_close( &_FS_lfs, &file );
    return FS_ERR_FAILED;
  }
  res = printMemory( &file, 0, fileSize - 1, 4, 16, readFromFile );
  if ( res != FS_ERR_OK ) {
    // CLS1_SendStr("ERROR while calling PrintMemory()\r\n", io->stdErr);
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR while calling PrintMemory()\r\n" );
  }
  lfs_file_close( &_FS_lfs, &file );
  return res;
}

