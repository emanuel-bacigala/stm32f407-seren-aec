/**
  ******************************************************************************
  * @file    LwIP/LwIP_UDP_Echo_Server/Src/udp_echoserver.c
  * @author  MCD Application Team
  * @version V1.3.2
  * @date    13-November-2015
  * @brief   UDP echo server
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "rw.h"
#include "audio.h"
#include "opus.h"
#include "clock.h"
#include "settings.h"


extern    OpusDecoder* decoder;
extern struct udp_pcb* sender_upcb;
extern struct ip4_addr sender_addr;
extern        uint16_t sender_port;
extern        uint32_t lastPacketReceiveTime;

// packet loss staticstics
uint32_t packetsReceived = 0;
uint32_t packetsLost = 0;
uint32_t lastSeqNum = 0;
float    lossRate = 0.0;


void udp_echoserver_receive_callback(void* arg, struct udp_pcb* upcb, struct pbuf* p, struct ip4_addr* addr, u16_t port);

void udp_echoserver_init(uint16_t port)
{
   /* Create a new UDP control block  */
   struct udp_pcb* upcb = udp_new();

   if (upcb) {
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      err_t err = udp_bind(upcb, IP_ADDR_ANY, port);
      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        udp_recv(upcb, (udp_recv_fn)udp_echoserver_receive_callback, NULL);
      }
   }
}


// Function to send a UDP message
void send_udp_message(struct udp_pcb* pcb, uint8_t* data, int dataLen)
{
    // Create a pbuf to hold the message data
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, dataLen, PBUF_RAM);
    if (p != NULL) {
        // Copy the message data into the pbuf payload
        memcpy(p->payload, data, dataLen);

        // Send the pbuf over the UDP connection
        udp_send(pcb, p);

        // Free the pbuf
        pbuf_free(p);
    }
}


void opusStreamDecode(OpusDecoder* decoder, uint8_t* buf, int bufLen)
{
    int16_t* samples;

    if (decoder == NULL)
        return;

    // LED statistics
    GPIOD->BSRR = GPIO_BSRR_BR12 | GPIO_BSRR_BR13 | GPIO_BSRR_BR14 /*| GPIO_BSRR_BR15*/;  // reset all leds
    int numFreeSpkBuffs = spkGetNumFreeSamplesBuffers();
    if (numFreeSpkBuffs == SPK_BUFFERS_NUM)        GPIOD->BSRR = GPIO_BSRR_BS14;  // underflow: RED
    else if (numFreeSpkBuffs == SPK_BUFFERS_NUM-1) GPIOD->BSRR = GPIO_BSRR_BS13;  // underWarn: ORANGE
/*    else if (numFreeSpkBuffs == 0)                   GPIOD->BSRR = GPIO_BSRR_BS15;*/  // overflow:  BLUE
    else                                             GPIOD->BSRR = GPIO_BSRR_BS12;  // ok:        GREEN
    // LED statistics

    if ((samples = spkGetSamplesBuffer()) != NULL) { // audio input buffer available
        int frameSize = opus_decode(decoder, buf, bufLen, samples, SPK_FRAME_SIZE, 0);
        spkReturnSamplesBuffer(SPK_CHANNELS*frameSize);
    }

#if 0
    // start DMA if not running and prebuffer complete (0-full, 1-nearly full, ... SPK_BUFFERS_NUM - empty)
    if (isSpkDmaRunning() != 1 && spkGetNumFreeSamplesBuffers() <= 1)
        spkDmaStart();
#endif
}

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */


void udp_echoserver_receive_callback(void* arg, struct udp_pcb* upcb, struct pbuf* p, struct ip4_addr* addr, u16_t port)
{
    uint16_t packet_ftype;

    if (p->len < 2) {
        pbuf_free(p);
        return;
    }

    packet_ftype  = read_be16(p->payload) & UDP_PACKET_MASK_FTYPE;

    if (packet_ftype == UDP_PACKET_FTYPE_CALL) {  // seren CALL packet
        sender_upcb         = upcb;
        sender_addr.addr    = addr->addr;
        sender_port         = port;
        pbuf_free(p);

        // min. pklen = 1
        struct pbuf *pp = pbuf_alloc(PBUF_TRANSPORT, 22 + 1, PBUF_RAM);
        uint8_t* d = pp->payload;

        write_be16(d, UDP_PACKET_FTYPE_TABLE);       d+=2;

        /* pklen */
        uint16_t pklen = 1;   // min. pklen = 1
        write_be16(d, (uint16_t)pklen);              d+=2;
        memset(d, 0, pklen);                         d+=pklen;

        /* table size: all nodes excluding the sender of the call request */
        uint16_t table_size = 1;
        write_be16(d, table_size);                   d+=2;

        /* my nick */
        const char nick[NICKLEN] = "stm32f407";
        memcpy(d, nick, NICKLEN);                    d+=NICKLEN;

        udp_connect(upcb, addr, port);
        udp_send(upcb, pp);
        udp_disconnect(upcb);
        pbuf_free(pp);

        // reset statistics
        packetsReceived = 0;
        packetsLost = 0;
        lastSeqNum = 0;
        lossRate = 0.0;

        return;
    } else if (sender_port != 0 && packet_ftype == UDP_PACKET_FTYPE_BYE) {
        sender_addr.addr    = 0;
        sender_port         = 0;
    } else if (sender_port != 0 && packet_ftype == UDP_PACKET_FTYPE_NOP) { // seren NOP packet - MIC muted
        lastPacketReceiveTime = HAL_GetTick();
    } else if (sender_port != 0 && p->len == 10 && packet_ftype == UDP_PACKET_FTYPE_RTTREQ) { // seren RTT packet
        write_be16(p->payload, UDP_PACKET_FTYPE_RTTANS);
        udp_send(upcb, p);  // we are already connected and streaming MIC over upcb so no need to call udp_connect() and udp_disconnect()
    } else if (sender_port != 0 && p->len == 6 && packet_ftype == UDP_PACKET_FTYPE_PLINFO) { // seren packet loss info packet
        write_be16(p->payload, UDP_PACKET_FTYPE_PLINFO);
        lossRate = 100.0f * packetsLost / (float)(packetsLost + packetsReceived);
        write_be32(p->payload+2, (uint32_t)(lossRate*10000));  // loss rate is 10k * percentage
        udp_send(upcb, p);  // we are already connected ...
    } else if(sender_port != 0 && p->len > UDP_PACKET_SEREN_AUDIO_HEADER_LEN && packet_ftype == UDP_PACKET_FTYPE_AUDIO) { // seren AUDIO packet
        lastPacketReceiveTime = HAL_GetTick();
        uint32_t seqNum = read_be32(p->payload+2);

        if (packetsReceived == 0 || seqNum > lastSeqNum) {
            if (packetsReceived != 0 && seqNum != lastSeqNum + 1) {
                packetsLost += (seqNum - (lastSeqNum + 1));
            }

            lastSeqNum = seqNum;
            packetsReceived++;

            opusStreamDecode(decoder, p->payload + UDP_PACKET_SEREN_AUDIO_HEADER_LEN, p->len - UDP_PACKET_SEREN_AUDIO_HEADER_LEN);
        } else {
            // dup or out of order packet received...
        }
    } else {  // received something else
#if 0
        /* Connect to the remote client */
        udp_connect(upcb, addr, port);

        // Tell the client that we have accepted it
        udp_send(upcb, p);

        // free the UDP connection, so we can accept new clients
        udp_disconnect(upcb);
#endif
    }

    /* Free the p buffer */
    pbuf_free(p);
}
