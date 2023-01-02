#include <uvos.h>

#if defined (  __GNUC__  ) /* GCC CS3 */
#include <sys/stat.h>
#endif

#include <errno.h>
// #undef errno
// extern int errno;


// Helper macro to mark unused parameters and prevent compiler warnings.
// Appends _UNUSED to the variable name to prevent accidentally using them.
#define _UNUSED_(x) x ## _UNUSED __attribute__((__unused__))

__attribute__( ( weak ) )
caddr_t _sbrk( int incr )
{
	extern char _estack; /* Defined in the linker script */
	extern char _Min_Stack_Size; /* Defined in the linker script */
	extern char _end; /* Defined by the linker */
	static char * heap_end = &_end ;
	char * prev_heap_end = heap_end;

	if ( heap_end + incr > ( char * )__get_MSP() ) {
		/* Heap and stack collision */
		errno = ENOMEM;
		return ( caddr_t ) - 1;
	}
	/* Ensure to keep minimun stack size defined in the linker script */
	if ( heap_end + incr >= ( char * )( &_estack - &_Min_Stack_Size ) ) {
		errno = ENOMEM;
		return ( caddr_t ) - 1;
	}

	heap_end += incr ;
	return ( caddr_t ) prev_heap_end ;
}

__attribute__( ( weak ) )
int _close( _UNUSED_( int file ) )
{
	return -1;
}

__attribute__( ( weak ) )
int _fstat( _UNUSED_( int file ), struct stat * st )
{
	st->st_mode = S_IFCHR ;
	return 0;
}

__attribute__( ( weak ) )
int _isatty( _UNUSED_( int file ) )
{
	return 1;
}

__attribute__( ( weak ) )
int _lseek( _UNUSED_( int file ), _UNUSED_( int ptr ), _UNUSED_( int dir ) )
{
	return 0;
}

__attribute__( ( weak ) )
int _read( _UNUSED_( int file ), _UNUSED_( char * ptr ), _UNUSED_( int len ) )
{
	return 0;
}

__attribute__( ( weak ) )
int _write( _UNUSED_( int file ), char * ptr, int len )
{
	return 0;
}

__attribute__( ( weak ) )
void _exit( _UNUSED_( int status ) )
{
	for ( ; ; ) ;
}

__attribute__( ( weak ) )
int _kill( _UNUSED_( int pid ), _UNUSED_( int sig ) )
{
	errno = EINVAL;
	return -1;
}

__attribute__( ( weak ) )
int _getpid( void )
{
	return 1;
}