/*
 *******************************************************************************
 * Copyright (c) 2020-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#if defined(ARDUINO_GENERIC_F769BGTX) || defined(ARDUINO_GENERIC_F769BITX) ||\
    defined(ARDUINO_GENERIC_F769NGHX) || defined(ARDUINO_GENERIC_F769NIHX) ||\
    defined(ARDUINO_GENERIC_F779BITX) || defined(ARDUINO_GENERIC_F779NIHX)
#include "pins_arduino.h"

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
WEAK void SystemClock_Config(void)
{
  /* SystemClock_Config can be generated by STM32CubeMX */
#warning "SystemClock_Config() is empty. Default clock at reset is used."
}

#endif /* ARDUINO_GENERIC_* */
