#include <stdint.h>

extern volatile uint32_t ticks;

void SysTick_Init(void);
void delay_ms(uint32_t t);
void initClockHse(void);
void initClockHsi(void);
uint32_t HAL_GetTick_raw();
