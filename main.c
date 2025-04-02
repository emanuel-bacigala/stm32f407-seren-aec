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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include "clock.h"
#include "audio.h"
#include "opus.h"

#include "eth_gpio.h"
#include "lwip.h"
#include "lwip/udp.h"  // Init udp sender
#include "udp_echoserver.h"
#include "rw.h"
#include "speex/speex_echo.h"
#include "settings.h"

#define ARM_MATH_CM4
#include "arm_math.h"
#include "fir_coeffs.h"


#define ETH_IRQ_MODE // ethernet interrupt or polling mode

#define STEPS			10
#define EXPANDED_BUFFER_SIZE	(MIC_PDM_FRAME_SIZE_BITS/STEPS)   // Every bit in I2S signal needs to be expanded out into a word
                                                                  // so we will process MIC PDM FRAME in STEPS to conserve memory
#define FIR_DECIMATION_FACTOR	64
#define DECIMATED_BUFFER_SIZE	(EXPANDED_BUFFER_SIZE / FIR_DECIMATION_FACTOR)
#define MIC_FRAME_SIZE		(STEPS*DECIMATED_BUFFER_SIZE)

CCMRAM arm_fir_decimate_instance_f32 decimator;
CCMRAM float32_t decimatorState[FIR_COEFFS_LEN + EXPANDED_BUFFER_SIZE - 1];
CCMRAM float32_t expandedBuffer[EXPANDED_BUFFER_SIZE];
CCMRAM float32_t decimatedBuffer[DECIMATED_BUFFER_SIZE];

CCMRAM int16_t pcmBuffer[MIC_FRAME_SIZE];     // MIC PCM frame data
CCMRAM int16_t pcmBufferSpk[MIC_FRAME_SIZE];  // SPK frame (mono) - for AEC storage


extern volatile uint32_t transferComplete;

uint32_t lastPacketReceiveTime;
struct udp_pcb* sender_upcb = NULL;
struct ip4_addr sender_addr = {0,};
uint16_t        sender_port = 0;

SpeexEchoState *st = NULL;
OpusDecoder* decoder = NULL;


void pdmExpand(float32_t *outBuff, const int16_t *inBuff, int size)
{
    uint16_t outWord = 0;

    for (int bit=0; bit < size; bit++) {
        if (bit % 16 == 0)
            outWord = inBuff[bit>>4];  // div by 16

        if (outWord & 0x8000)
            outBuff[bit] = INT16_MAX;
        else
            outBuff[bit] = INT16_MIN;

        outWord <<= 1;
    }
}


int main(void)
{
    int status;

    initClockHse();  // setup clocks
    //initClockHsi();  // setup clocks

    SysTick_Init();  // Enable the SysTick interrupt every 1ms or 1us

// init LEDs
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | GPIO_MODER_MODE15);
    GPIOD->MODER |= (GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0);
    GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED12_1 | GPIO_OSPEEDR_OSPEED13_1 | GPIO_OSPEEDR_OSPEED14_1 | GPIO_OSPEEDR_OSPEED15_1);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15);
    GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15);

// ETH INIT
    initEthGpio();
#ifdef ETH_IRQ_MODE
    extern ETH_HandleTypeDef heth;
    heth.Init.RxMode = ETH_RXINTERRUPT_MODE;
#endif
    // IP addresses initialization
    uint8_t IP_ADDRESS[4] = {192, 168, 12, 6,};
    uint8_t NETMASK_ADDRESS[4] = {255, 255, 255, 0,};
    uint8_t GATEWAY_ADDRESS[4] = {192, 168, 12, 1,};
    uint16_t PORT = 4321;

    // enable ETH periph
    RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACTXEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACRXEN;
    MX_LWIP_Init(IP_ADDRESS, NETMASK_ADDRESS, GATEWAY_ADDRESS);

    // Init udp echo server (listener)
    udp_echoserver_init(PORT);

#ifdef ETH_IRQ_MODE
    NVIC_EnableIRQ(ETH_IRQn);
    NVIC_SetPriority(ETH_IRQn, 63);
#endif
// ETH INIT

// AUDIO INIT
    if      (SPK_FREQUENCY == 48000) audioClkInit(258,6,42,0,3,1);   // 16kHz MIC, 48kHz SPEAKER
    else if (SPK_FREQUENCY == 24000) audioClkInit(258,6,42,0,7,0);   // 16kHz MIC, 24kHz SPEAKER
    else if (SPK_FREQUENCY == 16000) audioClkInit(258,6,42,0,10,1);  // 16kHz MIC, 16kHz SPEAKER

  // SPK
    spkGpioInit();
    spkCodecInit(NULL);
    spkCodecSetVolume(0xDF);
    spkDmaInit();

  // MIC
    micGpioInit();
    micDmaInit();
// AUDIO INIT

// Opus decoder init
#if 1
    decoder = opus_decoder_create(SPK_FREQUENCY, SPK_CHANNELS, &status);
    while (status < 0)
    {
        GPIOD->ODR ^= GPIO_BSRR_BS13;  // toggle orange LED on ERROR
        delay_ms(50);
    }
#endif

// Opus encoder init
#if 1
    OpusEncoder *encoder = opus_encoder_create(MIC_FREQUENCY, 1, /*OPUS_APPLICATION_VOIP*/OPUS_APPLICATION_RESTRICTED_LOWDELAY, &status);
    while (status < 0) {
        GPIOD->ODR ^= GPIO_BSRR_BS13;  // toggle orange LED on ERROR
        delay_ms(50);
    }

    //status = opus_encoder_ctl(encoder, OPUS_SET_BITRATE(BITRATE));
    status |= opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(0));
    status |= opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    while (status < 0) {
        GPIOD->ODR ^= GPIO_BSRR_BS13;  // toggle orange LED on ERROR
        delay_ms(50);
    }
#endif

// Speex echo cancel init
#if 1
    int samplerate = MIC_FREQUENCY;
    while ((st  = speex_echo_state_init(MIC_FRAME_SIZE, 480)) == NULL) {
        GPIOD->ODR ^= GPIO_BSRR_BS13;  // toggle orange LED on ERROR
        delay_ms(50);
    }
    speex_echo_ctl(st, SPEEX_ECHO_SET_SAMPLING_RATE, (int*)&samplerate);
#endif

    //arm_status armStatus;
    status = arm_fir_decimate_init_f32(&decimator, FIR_COEFFS_LEN, FIR_DECIMATION_FACTOR, firCoeffs, decimatorState, EXPANDED_BUFFER_SIZE);
    while (status != ARM_MATH_SUCCESS) {
        GPIOD->ODR ^= GPIO_BSRR_BS13;  // toggle orange LED on ERROR
        delay_ms(50);
    }

    int isConnected = 0;
    int doEchoCancel = 1;

    NVIC_DisableIRQ(SysTick_IRQn);
    while (1) {
        // wait for connection
        if (isConnected == 0) {
            while (sender_port == 0) __asm volatile("wfi");
            NVIC_EnableIRQ(SysTick_IRQn);
            udp_connect(sender_upcb, &sender_addr, sender_port);
            lastPacketReceiveTime = HAL_GetTick();
            isConnected = 1;
            micDmaStart();
            if (doEchoCancel & 1) GPIOD->BSRR = GPIO_BSRR_BS15;
        } else if (isConnected == 1 && (sender_port == 0 || (lastPacketReceiveTime && HAL_GetTick() - lastPacketReceiveTime > 5000))) {
            micDmaStop();
            //spkDmaStop();  // TODO: save energy when waiting for a call
            udp_disconnect(sender_upcb);
            sender_upcb = NULL;
            isConnected = 0;
            sender_port = 0;  // just in case of timeout
            GPIOD->BSRR = GPIO_BSRR_BR15;
            NVIC_DisableIRQ(SysTick_IRQn);
            continue;
        }

        if ((GPIOA->IDR & 1) == 1) {  // pressed
            doEchoCancel |= 2;
        } else if ((GPIOA->IDR & 1) == 0 && doEchoCancel & 2) {  // released after press
            doEchoCancel = doEchoCancel & 1 ? 0 : 1;
            if (doEchoCancel & 1) GPIOD->BSRR = GPIO_BSRR_BS15;
            else                  GPIOD->BSRR = GPIO_BSRR_BR15;
        }

        while (!transferComplete) __asm volatile("wfi");  // wait for MIC data
        transferComplete = 0;

#if 1  // An attemt to start MIC and SPK DMA simultaneously
        // start DMA if not running and prebuffer is complete (0-full, 1-nearly full, ... SPK_BUFFERS_NUM - empty)
        if (isSpkDmaRunning() != 1 && spkGetNumFreeSamplesBuffers() <= 1)
            spkDmaStart();
#endif

        // convert PDM to PCM
        int16_t* pdmBuffer = micGetPdmBuffer();
        int16_t* pdmBufferPtr = pdmBuffer;
        for (int i=0; i<STEPS; i++) {
            pdmExpand(expandedBuffer, &pdmBufferPtr[i*(MIC_PDM_FRAME_SIZE/STEPS)], EXPANDED_BUFFER_SIZE);
            arm_fir_decimate_f32(&decimator, expandedBuffer, decimatedBuffer, EXPANDED_BUFFER_SIZE);
            int16_t* pcmBufferPtr = &pcmBuffer[i*DECIMATED_BUFFER_SIZE];
            for (int j=0; j<DECIMATED_BUFFER_SIZE; j++) {
                pcmBufferPtr[j] = MIC_GAIN*(int16_t)decimatedBuffer[j];
            }
        }

        // perform AEC if requested
        int16_t *micBuffer = pcmBuffer;
        int16_t* spkBuffer = (int16_t*)DMA1_Stream7->M0AR;  // buffer that's just playing (OK for filterLen >= 480)
        if (doEchoCancel & 1 && isSpkDmaRunning() == 1) {
            for (int i=0; i<MIC_FRAME_SIZE; i++) pcmBufferSpk[i] = (spkBuffer[2*i] + spkBuffer[2*i+1])>>1; // stereo(SPK) -> mono(MIC)
            micBuffer = (int16_t*)&expandedBuffer[512];  // re-using expandedBuffer variable for echo free frame
            speex_echo_cancellation(st, pcmBuffer, pcmBufferSpk, micBuffer);
        }

        // encode (re-using expandedBuffer variable for opus encoded data)
        int nbBytes = opus_encode(encoder, micBuffer, MIC_FRAME_SIZE, ((uint8_t*)expandedBuffer) + UDP_PACKET_SEREN_AUDIO_HEADER_LEN,
                                  (sizeof(expandedBuffer) - UDP_PACKET_SEREN_AUDIO_HEADER_LEN)/2);
        while (nbBytes < 0) {
            GPIOD->ODR ^= GPIO_BSRR_BS13;  // toggle orange LED on ERROR
            delay_ms(50);
        }

        // send OPUS packet
        uint8_t* expandedBuffer8 = (uint8_t*)expandedBuffer;
        write_be16(expandedBuffer8, UDP_PACKET_FTYPE_AUDIO);  // write header

        static uint32_t seqNum = 0;
        write_be32(&expandedBuffer8[2], seqNum++);            // write seqNum

        send_udp_message(sender_upcb, expandedBuffer8, nbBytes + UDP_PACKET_SEREN_AUDIO_HEADER_LEN);

#ifndef ETH_IRQ_MODE
        MX_LWIP_Process();
#endif
    }
}


#ifdef ETH_IRQ_MODE
struct pbuf * low_level_input(struct netif *netif);
void ETH_IRQHandler(void)
{
    extern ETH_HandleTypeDef heth;
    extern struct netif gnetif;
    ETH_HandleTypeDef* _heth = &heth;
    struct netif* netif = &gnetif;
    struct pbuf* p;
    err_t err;

    /* Frame received */
    if (__HAL_ETH_DMA_GET_FLAG(_heth, ETH_DMA_FLAG_R))
    {
        while((p = low_level_input(netif)) != NULL)
        {
            /* entry point to the LwIP stack */
            err = netif->input(p, netif);

            if (err != ERR_OK)
            {
                LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
                pbuf_free(p);
            }
        }
        sys_check_timeouts();
    }

  /* Clear the interrupt flags. */
    __HAL_ETH_DMA_CLEAR_IT(_heth, ETH_DMA_IT_R);
    __HAL_ETH_DMA_CLEAR_IT(_heth, ETH_DMA_IT_T);
    __HAL_ETH_DMA_CLEAR_IT(_heth, ETH_DMA_IT_NIS);
    __HAL_ETH_DMA_CLEAR_IT(_heth, ETH_DMA_FLAG_AIS);
}
#endif


void HardFault_Handler(void)
{
    GPIOD->ODR ^= GPIO_BSRR_BS14;  // toggle RED led on ERROR
}
