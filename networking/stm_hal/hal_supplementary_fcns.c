
#include "stm32f407xx.h"


uint32_t HAL_RCC_GetHCLKFreq(void)
{
  return SystemCoreClock;
}
