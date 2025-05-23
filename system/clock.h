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

extern volatile uint32_t ticks;

void SysTick_Init(void);
void delay_ms(uint32_t t);
void initClockHse(void);
void initClockHsi(void);
uint32_t HAL_GetTick_raw();
