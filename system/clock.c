/*
 * Copyright (C) 2024 Matus Jurecka
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "settings.h"


volatile uint32_t ticks = 0;

//#define SYSTICK_1US
#define SYSTICK_1MS

// Call this after setting up the clock
void SysTick_Init(void)
{
#if defined(SYSTICK_1MS)
    // Enable the SysTick interrupt every 1ms
    SysTick_Config(SystemCoreClock / 1000);
#elif defined(SYSTICK_1US)
    // Enable the SysTick interrupt every 1us
    SysTick_Config(SystemCoreClock / 1000000);
#else
    #error "systick interval undefined"
#endif
    NVIC_EnableIRQ(SysTick_IRQn);
}

uint32_t HAL_GetTick()
{
#if defined(SYSTICK_1MS)
    return ticks;
#elif defined(SYSTICK_1US)
    return ticks/1000;
#endif
}

uint32_t HAL_GetTick_raw()
{
    return ticks;
}


void delay_ms(uint32_t t)
{
    uint32_t elapsed;
    uint32_t start = HAL_GetTick();

    do
    {
        elapsed = HAL_GetTick() - start;
    } while (elapsed < t);
}


void SysTick_Handler(void)
{
    ticks++;
}

#define HAL_MAX_DELAY      0xFFFFFFFFU  // form stm32f4xx_hal_def.h
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  // Add a freq to guarantee minimum wait
  if (wait < HAL_MAX_DELAY)
  {
    wait++;
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
  }
}


void initClockHse(void)
{
    /* Flash settings (see RM0090 rev9, p80) */
    FLASH->ACR =
            FLASH_ACR_LATENCY_5WS               /* 6 CPU cycle wait */
          | FLASH_ACR_PRFTEN                    /* enable prefetch */
          | FLASH_ACR_ICEN                      /* instruction cache enable */
          | FLASH_ACR_DCEN;                     /* data cache enable */

    /* Enable HSE at 8 MHz (na Discovery) */
    RCC->CR = RCC_CR_HSEON;
    while((RCC->CR & RCC_CR_HSERDY) == 0);      /* wait for HSE ready */

    /* Disable PLL */
    RCC->CR &= ~RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY);


    /*  PLL config
     *  crystal:  8MHz
     *  M = 4     (8/4   =   2MHz) - 2,3,...63			(  1 -   2 MHz) - PLL input frequency
     *  N = 168   (2*168 = 336MHz) - 2,3,...432,434,...510	(100 - 432 MHz) - VCO frequency
                        asi musi platit : 50 <= PLLN <= 432
     *  P = 0     (336/2 = 168MHz) - 00:2,01:4,10:6,11:8	(    < 180 MHz) - main system clock
     *  Q = 7     (336/7 =  48MHz) - 2,3,...15			(    =  48 MHz) - USB clock
     *  R = ?     (nepouzivam)     - 2...7			(no constrains?)- I2S, SAI, SYSTEMa and SPDIF-Rx clock
     */
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC_Msk |
                      RCC_PLLCFGR_PLLM_Msk   |
                      RCC_PLLCFGR_PLLN_Msk   |
                      RCC_PLLCFGR_PLLP_Msk   |
                      RCC_PLLCFGR_PLLQ_Msk);              /* clear affected bits */
    RCC->PLLCFGR |= ( RCC_PLLCFGR_PLLSRC_HSE            | /* PLL source */
                      4U << RCC_PLLCFGR_PLLM_Pos        |
                      CPU_SPEED << RCC_PLLCFGR_PLLN_Pos | /* up to 250 */
                      0U << RCC_PLLCFGR_PLLP_Pos        |
                      7U << RCC_PLLCFGR_PLLQ_Pos );

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0);


    /*  Configure clocks
     *  Max SYSCLK: 168MHz
     *  Max AHB:  SYSCLK
     *  Max APB1: SYSCLK/4 = 48MHz
     *  Max APB2: SYSCLK/2 = 86MHz
     *  + enable PLLI2S output MCO2 with clock divider=1
     */
    RCC->CFGR &= ( RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk | RCC_CFGR_I2SSRC_Msk |
                   RCC_CFGR_MCO2PRE_Msk | RCC_CFGR_MCO2_Msk | RCC_CFGR_SW_Msk );   /* clear affected bits */
    RCC->CFGR |= ( RCC_CFGR_SW_PLL              | /* PLL clock source: 00=HSI, 01=HSE, 10=PLL */
                   0x00 << RCC_CFGR_MCO2_Pos    | /* Clock output 2 00:sysclk, 01:plli2s, 10:hse, 11:pll */
                   0x06 << RCC_CFGR_MCO2PRE_Pos | /* Clock output divider 0xx:div1, 100:div2, 101:div3, 110:div4, 111:div5 */
                   0x00 << RCC_CFGR_I2SSRC_Pos  | /* I2S clock surce 0:PLLI2S, 1:external clock on I2S_CLKIN */
                   RCC_CFGR_PPRE2_DIV2          | /* APB2 prescaler */
                   RCC_CFGR_PPRE1_DIV4 );         /* APB2 prescaler */

    while((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);  /* Wait for SYSCLK to be PPL */

    SystemCoreClockUpdate();
}


void initClockHsi(void)
{
    /* Flash settings (see RM0090 rev9, p80) */
    FLASH->ACR =
            FLASH_ACR_LATENCY_5WS               /* 6 CPU cycle wait */
          | FLASH_ACR_PRFTEN                    /* enable prefetch */
          | FLASH_ACR_ICEN                      /* instruction cache enable */
          | FLASH_ACR_DCEN;                     /* data cache enable */


    /* Enable HSI at 16 MHz */
    RCC->CR |= RCC_CR_HSION;
    while((RCC->CR & RCC_CR_HSIRDY) == 0);              /* wait for HSI ready */
    uint8_t HSITrim = 15;                               /* 5 bits: 0 - 31 */  // 16 - too fast; 15 - too slow
    RCC->CR &= ~RCC_CR_HSITRIM_Msk;                     /* clear HSI trim bits */
    RCC->CR |= (HSITrim & 0x1F) << RCC_CR_HSITRIM_Pos;  /* set HSITRIM[4:0] in CR */

    /* Disable PLL */
    RCC->CR &= ~RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY);

    /*  PLL config
     *  crystal:  16MHz
     *  M = 8     (16/8  =   2MHz) - 2,3,...63			(  1 -   2 MHz)  - PLL input frequency
     *  N = 168   (2*168 = 336MHz) - 2,3,...432,434,...510	(100 - 432 MHz)  - VCO frequency
                        asi musi platit : 50 <= PLLN <= 432
     *  P = 0     (336/2 = 168MHz) - 00:2,01:4,10:6,11:8	( < 180 MHz)     - main system clock
     *  Q = 7     (336/7 =  48MHz) - 2,3,...15			( =  48 MHz)     - USB clock
     *  R = ?     (nepouzivam)     - 2...7			(no constrains?) - I2S, SAI, SYSTEMa and SPDIF-Rx clock
     */
    RCC->PLLCFGR &= ~( RCC_PLLCFGR_PLLSRC_Msk |
                       RCC_PLLCFGR_PLLM_Msk   |
                       RCC_PLLCFGR_PLLN_Msk   |
                       RCC_PLLCFGR_PLLP_Msk   |
                       RCC_PLLCFGR_PLLQ_Msk);              /* clear affected bits */
    RCC->PLLCFGR |=  ( RCC_PLLCFGR_PLLSRC_HSI            | /* PLL source */
                       8U << RCC_PLLCFGR_PLLM_Pos        |
                       CPU_SPEED << RCC_PLLCFGR_PLLN_Pos | /* up to 250 */
                       0U << RCC_PLLCFGR_PLLP_Pos        |
                       7U << RCC_PLLCFGR_PLLQ_Pos );

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    /*  Configure clocks
     *  Max SYSCLK: 168MHz
     *  Max AHB:  SYSCLK
     *  Max APB1: SYSCLK/4 = 48MHz
     *  Max APB2: SYSCLK/2 = 86MHz
     *  + enable PLLI2S output MCO2 with clock divider=1
     */
    RCC->CFGR &= ( RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk | RCC_CFGR_I2SSRC_Msk |
                   RCC_CFGR_MCO2PRE_Msk | RCC_CFGR_MCO2_Msk | RCC_CFGR_SW_Msk );   /* clear affected bits */
    RCC->CFGR |= ( RCC_CFGR_SW_PLL              | /* PLL clock source: 00=HSI, 01=HSE, 10=PLL */
                   0x00 << RCC_CFGR_MCO2_Pos    | /* Clock output 2 is SYSCLK (RCC_CFGR_MCO2) 00:sysclk, 01:plli2s, 10:hse, 11:pll */
                   0x06 << RCC_CFGR_MCO2PRE_Pos | /* Clock output divider 0xx:div1, 100:div2, 101:div3, 110:div4, 111:div5 */
                   0x00 << RCC_CFGR_I2SSRC_Pos  | /* I2S clock surce 0:PLLI2S, 1:external clock on I2S_CLKIN */
                   RCC_CFGR_PPRE2_DIV2          | /* APB2 prescaler */
                   RCC_CFGR_PPRE1_DIV4 );         /* APB2 prescaler */

    while((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);  /* Wait for SYSCLK to be PPL */

    SystemCoreClockUpdate();
}
