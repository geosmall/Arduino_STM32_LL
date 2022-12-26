#ifndef UVOS_RTC_H
#define UVOS_RTC_H

/* Public Functions */
extern uint32_t UVOS_RTC_Counter();
extern float UVOS_RTC_Rate();
extern float UVOS_RTC_MsPerTick();
extern bool UVOS_RTC_RegisterTickCallback(void (*fn)(uint32_t id), uint32_t data);
extern time_t UW_RTC__get_timestamp( void );

#endif /* UVOS_RTC_H */
