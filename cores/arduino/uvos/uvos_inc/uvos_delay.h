#ifndef UVOS_DELAY_H
#define UVOS_DELAY_H

/* Public Functions */
extern int32_t UVOS_DELAY_Init(void);
extern int32_t UVOS_DELAY_WaituS(uint32_t uS);
extern int32_t UVOS_DELAY_WaitmS(uint32_t mS);
extern uint32_t UVOS_DELAY_GetuS();
extern uint32_t UVOS_DELAY_GetuSSince(uint32_t t);
extern uint32_t UVOS_DELAY_GetRaw();
extern uint32_t UVOS_DELAY_DiffuS(uint32_t raw);

#endif /* UVOS_DELAY_H */
