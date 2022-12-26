/* HAL raised several warnings, ignore them */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#ifdef STM32F4xx
  #include "uvos_sys.c"
#elif STM32F7xx
  #include "uvos_sys.c"
#elif STM32H7xx
  #include "uvos_sys.c"
#else
  #error "Architecture not supported"
#endif

#pragma GCC diagnostic pop
