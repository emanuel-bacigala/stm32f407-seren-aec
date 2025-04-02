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


void audioClkInit(int plln, int pllr, int i2s2div, int i2s2odd, int i2s3div, int i2s3odd);

void micGpioInit();
void micDmaInit();
void micDmaStart();
void micDmaStop();
int16_t* micGetPdmBuffer();

void spkGpioInit();
void spkDmaInit();
void spkDmaStart();
void spkDmaStop();
int16_t* spkGetSamplesBuffer();
void spkReturnSamplesBuffer(uint16_t audioBufferFilledLen);
int spkGetNumFreeSamplesBuffers();
int isSpkDmaRunning();
void spkCodecInit(void* callBackFunction);
void spkCodecSetVolume(int volume);
void spkCodecOn();
void spkCodecOff();
void (*spkCallBackFunction)(int16_t* audioBuffer, uint16_t* audioBufferLen);
