#include <UAVWare.h>
#include "ff.h"
#include "printf.h"

//========================================================================================================================//
//                                                    IO SETUP                                                            //
//========================================================================================================================//

extern "C" int putchar_( char c )
{
#if defined( UVOS_COM_DEBUG )
  UVOS_COM_SendChar( UVOS_COM_DEBUG, c );
#endif // defined( UVOS_COM_DEBUG )
  return 0;
}

extern "C" char getchar_( void )
{
  char c;
#if defined( UVOS_COM_DEBUG )
  /* getchar is a blocking fcn - wait forever for a character */
  while ( UVOS_COM_RX_Data_Available( UVOS_COM_DEBUG ) == 0 );
  /* UVOS_COM_ReceiveChar() returns -1 on errer */
  int32_t res = UVOS_COM_ReceiveChar( UVOS_COM_DEBUG, &c );
  if ( res >= 0 ) {
    return c;
  }
#endif // defined( UVOS_COM_DEBUG )
  return EOF;
}

int puts_( const char *s )
{
  while ( *s ) {
    if ( putchar_( *s++ ) == EOF )
      return EOF;
  }

  if ( putchar_( '\r' ) == EOF )
    return EOF;
  if ( putchar_( '\n' ) == EOF )
    return EOF;

  return 0;
}

//========================================================================================================================//
//                                                   Application                                                          //
//========================================================================================================================//

#if 0 // GLS

FRESULT f_mount ( FATFS *fs, const TCHAR *path, BYTE opt );   /* Mount/Unmount a logical drive */
FRESULT f_open ( FIL *fp, const TCHAR *path, BYTE mode );     /* Open or create a file */
FRESULT f_close ( FIL *fp );                    /* Close an open file object */
FRESULT f_read ( FIL *fp, void *buff, UINT btr, UINT *br );   /* Read data from the file */
FRESULT f_write ( FIL *fp, const void *buff, UINT btw, UINT *bw ); /* Write data to the file */

int lfs_mount( lfs_t *lfs, const struct lfs_config *config );
int lfs_file_open( lfs_t *lfs, lfs_file_t *file, const char *path, int flags );
int lfs_file_opencfg( lfs_t *lfs, lfs_file_t *file, const char *path, int flags, const struct lfs_file_config *config );
int lfs_file_close( lfs_t *lfs, lfs_file_t *file );
lfs_ssize_t lfs_file_read( lfs_t *lfs, lfs_file_t *file, void *buffer, lfs_size_t size );
lfs_ssize_t lfs_file_write( lfs_t *lfs, lfs_file_t *file, const void *buffer, lfs_size_t size );

#endif // GLS

/* Buffers used for displaying Time and Date */
uint8_t aShowTime[50] = {0};
uint8_t aShowDate[50] = {0};

volatile uint32_t Timer;

DWORD get_fattime ( void )
{
  return ( ( DWORD )( FF_NORTC_YEAR - 1980 ) << 25 | ( DWORD )FF_NORTC_MON << 21 | ( DWORD )FF_NORTC_MDAY << 16 );
}


void setup()
{

  /* Brings up System using CMSIS functions, initializes periph clock, gpio pins. */
  UVOS_SYS_Init();

  /* board driver init */
  if ( UVOS_Board_Init() ) {
#ifdef UVOS_INCLUDE_DEBUG_CONSOLE
    UVOS_DEBUG_Panic( "System initialization Error\r\n" );
#endif // UVOS_INCLUDE_DEBUG_CONSOLE
  } else {
#ifdef UVOS_INCLUDE_DEBUG_CONSOLE
    UVOS_COM_SendString( UVOS_COM_DEBUG, "System initialized\r\n" );
#endif // UVOS_INCLUDE_DEBUG_CONSOLE
  }

  printf_( "FatFS SD card demo\r\n" );
  printf_( FF_USE_LFN ? "LFN Enabled" : "LFN Disabled" );
  printf_( ", Code page: %u\r\n", FF_CODE_PAGE );

  //some variables for FatFs
  FATFS FatFs;  //Fatfs handle
  FIL fil;    //File handle
  FRESULT fres; //Result after operations

  //Open the file system
  fres = f_mount( &FatFs, "", 1 ); //1=mount now
  if ( fres != FR_OK ) {
    printf_( "f_mount error (%i)\r\n", fres );
    while ( 1 );
  }

  UVOS_DELAY_WaitmS( 100 );

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS *getFreeFs;

  fres = f_getfree( "", &free_clusters, &getFreeFs );
  if ( fres != FR_OK ) {
    printf_( "f_getfree error (%i)\r\n", fres );
    while ( 1 );
  }

  //Formula comes from ChaN's documentation
  total_sectors = ( getFreeFs->n_fatent - 2 ) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  printf_( "SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2 );

  //Now let's try to open file "test.txt"
  fres = f_open( &fil, "test.txt", FA_READ );
  if ( fres != FR_OK ) {
    printf_( "f_open error (%i)\r\n", fres );
    while ( 1 );
  }
  printf_( "Able to open 'test.txt' for reading!\r\n" );

  //Read 30 bytes from "test.txt" on the SD card
  BYTE readBuf[30];

  //We can either use f_read OR f_gets to get data out of files
  //f_gets is a wrapper on f_read that does some string formatting for us
  TCHAR *rres = f_gets( ( TCHAR * )readBuf, 30, &fil );
  if ( rres != 0 ) {
    printf_( "Read string from 'test.txt' contents: %s\r\n", readBuf );
  } else {
    printf_( "f_gets error (%i)\r\n", fres );
  }

  // Be tidy - don't forget to close your file!
  f_close( &fil );

  // Try and write a file "write.txt"
  fres = f_open( &fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS );
  if ( fres == FR_OK ) {
    printf_( "I was able to open 'write.txt' for writing\r\n" );
  } else {
    printf_( "f_open error (%i)\r\n", fres );
  }

  // Copy in a string
  memcpy( ( char * )readBuf, "a new file is made!", 19 );
  UINT bytesWrote;
  fres = f_write( &fil, readBuf, 19, &bytesWrote );
  if ( fres == FR_OK ) {
    printf_( "Wrote %i bytes to 'write.txt'!\r\n", bytesWrote );
  } else {
    printf_( "f_write error (%i)\r\n", fres );
  }

  // Be tidy - don't forget to close your file!
  f_close( &fil );

  // We're done, so de-mount the drive
  f_mount( NULL, "", 0 );

  printf_( "All done!\r\n" );
}

void loop()
{

}