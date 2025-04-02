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

#include "stm32f407xx.h"


void initEthGpio(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1
    */

    GPIOC->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
    GPIOC->MODER |= GPIO_MODER_MODE1_1 |
                    GPIO_MODER_MODE4_1 |
                    GPIO_MODER_MODE5_1;                                 // PC1,PC4,PC5 in AF mode
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_0 | GPIO_OSPEEDR_OSPEED1_1 | // high speed
                      GPIO_OSPEEDR_OSPEED4_0 | GPIO_OSPEEDR_OSPEED4_1 |
                      GPIO_OSPEEDR_OSPEED5_0 | GPIO_OSPEEDR_OSPEED5_1;

    GPIOC->AFR[0] |= (11U << GPIO_AFRL_AFSEL1_Pos) |
                     (11U << GPIO_AFRL_AFSEL4_Pos) |
                     (11U << GPIO_AFRL_AFSEL5_Pos); // AF11 for PC1,PC4,PC5

    GPIOA->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE7);
    GPIOA->MODER |= GPIO_MODER_MODE1_1 |
                    GPIO_MODER_MODE2_1 |
                    GPIO_MODER_MODE7_1;                                 // PA1,PA2,PA7 in AF mode
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_0 | GPIO_OSPEEDR_OSPEED1_1 | // high speed
                      GPIO_OSPEEDR_OSPEED2_0 | GPIO_OSPEEDR_OSPEED2_1 |
                      GPIO_OSPEEDR_OSPEED7_0 | GPIO_OSPEEDR_OSPEED7_1;

    GPIOA->AFR[0] |= (11U << GPIO_AFRL_AFSEL1_Pos) |
                     (11U << GPIO_AFRL_AFSEL2_Pos) |
                     (11U << GPIO_AFRL_AFSEL7_Pos); // AF11 for PA1,PA2,PA7

    GPIOB->MODER &= ~(GPIO_MODER_MODE11 | GPIO_MODER_MODE12 | GPIO_MODER_MODE13);
    GPIOB->MODER |= GPIO_MODER_MODE11_1 |
                    GPIO_MODER_MODE12_1 |
                    GPIO_MODER_MODE13_1;                                  // PB11,PB12,PB13 in AF mode
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED11_0 | GPIO_OSPEEDR_OSPEED11_1 | // high speed
                      GPIO_OSPEEDR_OSPEED12_0 | GPIO_OSPEEDR_OSPEED12_1 |
                      GPIO_OSPEEDR_OSPEED13_0 | GPIO_OSPEEDR_OSPEED13_1;

    GPIOB->AFR[1] |= (11U << GPIO_AFRH_AFSEL11_Pos) |
                     (11U << GPIO_AFRH_AFSEL12_Pos) |
                     (11U << GPIO_AFRH_AFSEL13_Pos); // AF11 for PB11,PB12,PB13
}
