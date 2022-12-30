#include <uvos.h>
#include <time.h>

#ifdef UVOS_INCLUDE_RTC

#include <uvos_rtc_priv.h>

static uint32_t _WUT_reload;
#define RTC_TIMEOUT_VALUE       1000

typedef enum {
  RTC_OK       = 0x00U,
  RTC_ERROR    = 0x01U,
  RTC_BUSY     = 0x02U,
  RTC_TIMEOUT  = 0x03U
} RTC_StatusTypeDef;

// External interrupt line 22 Connected to the RTC Wake-up event
#define EXTI_LINE_WAKEUPTIMER_EVENT     LL_EXTI_LINE_22

struct rtc_callback_entry {
  void     ( *fn )( uint32_t );
  uint32_t data;
};

#define UVOS_RTC_MAX_CALLBACKS 3
struct rtc_callback_entry rtc_callback_list[ UVOS_RTC_MAX_CALLBACKS ];
static uint8_t rtc_callback_next = 0;

static uint32_t SetWakeUpTimer_IT( RTC_TypeDef * RTCx, uint32_t WakeUpCounter, uint32_t WakeUpClock )
{
  __IO uint32_t count;

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection( RTCx );

  /* Check RTC WUTWF flag is reset only when wake up timer enabled */
  if ( ( RTCx->CR & RTC_CR_WUTE ) != RESET ) {
    /* Wait till RTC WUTWF flag is reset and if Time out is reached exit */
    count = RTC_TIMEOUT_VALUE  * ( SystemCoreClock / 32U / 1000U );
    do {
      if ( count-- == 0U ) {
        /* Enable the write protection for RTC registers */
        LL_RTC_EnableWriteProtection( RTCx );

        return RTC_TIMEOUT;
      }
    } while ( LL_RTC_IsActiveFlag_WUTW( RTCx ) == SET );
  }

  /* Disable the Wake-up Timer */
  LL_RTC_WAKEUP_Disable( RTCx );

  /* Wait till RTC WUTWF flag is set and if Time out is reached exit */
  count = RTC_TIMEOUT_VALUE  * ( SystemCoreClock / 32U / 1000U );
  do {
    if ( count-- == 0U ) {
      /* Enable the write protection for RTC registers */
      LL_RTC_EnableWriteProtection( RTCx );

      return RTC_TIMEOUT;
    }
  } while ( LL_RTC_IsActiveFlag_WUTW( RTCx ) == RESET );

  /* Configure the Wake-up Timer counter */
  RTCx->WUTR = ( uint32_t )WakeUpCounter;

  /* Clear the Wake-up Timer clock source bits in CR register */
  RTCx->CR &= ( uint32_t )~RTC_CR_WUCKSEL;

  /* Configure the clock source */
  RTCx->CR |= ( uint32_t )WakeUpClock;

  /* RTC WakeUpTimer Interrupt Configuration: EXTI configuration */
  LL_EXTI_EnableIT_0_31( EXTI_LINE_WAKEUPTIMER_EVENT );
  LL_EXTI_EnableRisingTrig_0_31( EXTI_LINE_WAKEUPTIMER_EVENT );

  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT( RTCx );

  /* Configure the Interrupt in the RTC_CR register */
  LL_RTC_EnableIT_WUT( RTCx );

  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Enable( RTCx );

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection( RTCx );

  return RTC_OK;
}

#if 0 // GLS

static uint32_t DeactivateWakeUpTimer( RTC_TypeDef * RTCx )
{
  __IO uint32_t count;

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection( RTCx );

  /* Disable the Wake-up Timer */
  LL_RTC_WAKEUP_Disable( RTCx );

  /* In case of interrupt mode is used, the interrupt source must disabled */
  LL_RTC_DisableIT_WUT( RTCx );

  /* Wait till RTC WUTWF flag is set and if Time out is reached exit */
  while ( LL_RTC_IsActiveFlag_WUTW( RTCx ) == RESET ) {
    count = RTC_TIMEOUT_VALUE  * ( SystemCoreClock / 32U / 1000U );

    if ( count-- == 0U ) {
      /* Enable the write protection for RTC registers */
      LL_RTC_EnableWriteProtection( RTCx );

      return RTC_TIMEOUT;
    }
  }

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection( RTCx );

  return RTC_OK;
}

void UVOS_RTC_Init( const struct uvos_rtc_cfg * cfg )
{
  // RCC_BackupResetCmd(ENABLE);
  // RCC_BackupResetCmd(DISABLE);
  LL_RCC_ForceBackupDomainReset();
  LL_RCC_ReleaseBackupDomainReset();

  // RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR, ENABLE );
  LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_PWR );
  // PWR_BackupAccessCmd( ENABLE );
  LL_PWR_EnableBkUpAccess();

  // Divide external 8 MHz clock to 1 MHz
  // RCC_RTCCLKConfig( cfg->clksrc );
  LL_RCC_SetRTCClockSource( LL_RCC_RTC_CLKSOURCE_HSE );
  LL_RCC_SetRTC_HSEPrescaler( cfg->hse_clkdiv );

  // RCC_RTCCLKCmd( ENABLE );
  LL_RCC_EnableRTC();

  // RTC_WakeUpCmd( DISABLE );
  LL_RTC_WAKEUP_Disable( RTC );

  while ( LL_RTC_IsActiveFlag_WUTW( RTC ) != 1 ) {}
  // Divide 1 Mhz clock by 16 -> 62.5 khz
  // RTC_WakeUpClockConfig( RTC_WakeUpClock_RTCCLK_Div16 );
  LL_RTC_WAKEUP_SetClock( RTC, LL_RTC_WAKEUPCLOCK_DIV_8 );
  // Divide 62.5 khz by 200 to get 625 Hz
  // RTC_SetWakeUpCounter( cfg->prescaler ); // cfg->prescaler);
  LL_RTC_WAKEUP_SetAutoReload( RTC, cfg->WUT_reload );
  // RTC_WakeUpCmd( ENABLE );
  LL_RTC_WAKEUP_Enable( RTC );

  /* Configure and enable the RTC Wakeup interrupt */
  /* RTC Wakeup event is connected to EXTI line 22 */
  LL_EXTI_InitTypeDef ExtiInit = {
    .Line_0_31    = LL_EXTI_LINE_22,
    .LineCommand = ENABLE,
    .Mode = LL_EXTI_MODE_IT,
    .Trigger = LL_EXTI_TRIGGER_RISING,
  };
  LL_EXTI_Init( &ExtiInit );
  NVIC_Init( ( NVIC_InitTypeDef * ) &cfg->irq.init );
  // RTC_ITConfig( RTC_IT_WUT, ENABLE );
  LL_RTC_EnableIT_WUT( RTC );

  // RTC_ClearFlag( RTC_FLAG_WUTF );
  LL_RTC_ClearFlag_WUT( RTC );
}

#endif // GLS


void UVOS_RTC_Init( const struct uvos_rtc_cfg * cfg )
{

  /* Save local copies of clock cfg value */
  _WUT_reload = cfg->WUT_reload;

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  LL_RTC_TimeTypeDef RTC_TimeStruct = {0};
  LL_RTC_DateTypeDef RTC_DateStruct = {0};

  if ( LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_HSE ) {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource( LL_RCC_RTC_CLKSOURCE_HSE );
  }
  LL_RCC_SetRTC_HSEPrescaler( cfg->hse_clkdiv ); // Divide to get RTCCLK = 1 MHz

  /* Peripheral clock enable */
  LL_RCC_EnableRTC();

  /* Initialize RTC and set the Time and Date */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 124;
  RTC_InitStruct.SynchPrescaler = 7999;
  LL_RTC_Init( RTC, &RTC_InitStruct );
  // LL_RTC_SetAsynchPrescaler( RTC, 124 );
  // LL_RTC_SetSynchPrescaler( RTC, 7999 );

  /* Initialize RTC and set the Time and Date */
  if ( LL_RTC_BAK_GetRegister( RTC, LL_RTC_BKP_DR0 ) != 0x32F2 ) {
    RTC_TimeStruct.Hours = 0;
    RTC_TimeStruct.Minutes = 0;
    RTC_TimeStruct.Seconds = 0;
    LL_RTC_TIME_Init( RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct );
    RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_MONDAY;
    RTC_DateStruct.Month = LL_RTC_MONTH_JANUARY;
    RTC_DateStruct.Year = 0;
    LL_RTC_DATE_Init( RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct );
    LL_RTC_BAK_SetRegister( RTC, LL_RTC_BKP_DR0, 0x32F2 );
  }

  /* Enable the RTC WakeUp function */
  LL_RTC_WAKEUP_SetClock( RTC, cfg->WUT_clkdiv ); // Divide 1 Mhz RTCCLK by 16 -> 62.5 khz
  LL_RTC_WAKEUP_SetAutoReload( RTC, cfg->WUT_reload ); // Every 100 WUT cycles gives 625 Hz

  LL_EXTI_InitTypeDef EXTI_InitStruct = {
    .Line_0_31 = EXTI_LINE_WAKEUPTIMER_EVENT,
    .LineCommand = ENABLE,
    .Mode = LL_EXTI_MODE_IT,
    .Trigger = LL_EXTI_TRIGGER_RISING,
  };
  LL_EXTI_Init( &EXTI_InitStruct );
  /* RTC interrupt Init moved here */
  // NVIC_SetPriority( RTC_WKUP_IRQn, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), 0, 0 ) );
  // NVIC_EnableIRQ( RTC_WKUP_IRQn );
  NVIC_Init( ( NVIC_InitTypeDef * ) &cfg->irq.init );

  if ( SetWakeUpTimer_IT( RTC, cfg->WUT_reload, cfg->WUT_clkdiv ) != RTC_OK ) {
    UVOS_Assert( 0 );
  }

}

uint32_t UVOS_RTC_Counter()
{
  // return RTC_GetWakeUpCounter();
  return LL_RTC_WAKEUP_GetAutoReload( RTC );
}

/* FIXME: This shouldn't use hard-coded clock rates, dividers or prescalers.
 *        Should get these from the cfg struct passed to init.
 *        NOTE: In func below,
 *          8e6 = HSE
 *          128 = (LL_RCC_RTC_HSE_DIV_8 * LL_RTC_WAKEUPCLOCK_DIV_16)
 */
float UVOS_RTC_Rate()
{
  return ( float )( 8e6f / 128.0f ) / ( 1 + _WUT_reload );
}

float UVOS_RTC_MsPerTick()
{
  return 1000.0f / UVOS_RTC_Rate();
}

/* TODO: This needs a mutex around rtc_callbacks[] */
bool UVOS_RTC_RegisterTickCallback( void ( *fn )( uint32_t id ), uint32_t data )
{
  struct rtc_callback_entry * cb;

  if ( rtc_callback_next >= UVOS_RTC_MAX_CALLBACKS ) {
    return false;
  }

  cb = &rtc_callback_list[rtc_callback_next++];

  cb->fn   = fn;
  cb->data = data;
  return true;
}

void UVOS_RTC_irq_handler( void )
{
  // if ( RTC_GetITStatus( RTC_IT_WUT ) ) {
  if ( ( LL_RTC_IsEnabledIT_WUT( RTC ) != RESET ) && ( LL_RTC_IsActiveFlag_WUT( RTC ) != RESET ) ) {
    /* Call all registered callbacks */
    for ( uint8_t i = 0; i < rtc_callback_next; i++ ) {
      struct rtc_callback_entry * cb = &rtc_callback_list[i];
      if ( cb->fn ) {
        ( cb->fn )( cb->data );
      }
    }

    /* Clear the RTC Wakeup interrupt */
    // RTC_ClearITPendingBit( RTC_IT_WUT );
    LL_RTC_ClearFlag_WUT( RTC );
  }

  // if ( EXTI_GetITStatus( EXTI_Line22 ) != RESET ) {
  if ( LL_EXTI_IsActiveFlag_0_31( EXTI_LINE_WAKEUPTIMER_EVENT ) != RESET ) {
    // EXTI_ClearITPendingBit( EXTI_Line22 );
    LL_EXTI_ClearFlag_0_31( EXTI_LINE_WAKEUPTIMER_EVENT );
  }
}

time_t UW_RTC__get_timestamp( void )
{
  time_t timestamp;
  struct tm currTime;

  /* Note: need to convert to decimal value using __LL_RTC_CONVERT_BCD2BIN helper macro */
  currTime.tm_hour = __LL_RTC_CONVERT_BCD2BIN( LL_RTC_TIME_GetHour( RTC ) );
  currTime.tm_min = __LL_RTC_CONVERT_BCD2BIN( LL_RTC_TIME_GetMinute( RTC ) );
  currTime.tm_sec = __LL_RTC_CONVERT_BCD2BIN( LL_RTC_TIME_GetSecond( RTC ) );
  currTime.tm_mon = __LL_RTC_CONVERT_BCD2BIN( LL_RTC_DATE_GetMonth( RTC ) );
  currTime.tm_mday = __LL_RTC_CONVERT_BCD2BIN( LL_RTC_DATE_GetDay( RTC ) );
  currTime.tm_year = __LL_RTC_CONVERT_BCD2BIN( LL_RTC_DATE_GetYear( RTC ) );

  timestamp = mktime( &currTime );
  return timestamp;
}

#endif /* UVOS_INCLUDE_RTC */

