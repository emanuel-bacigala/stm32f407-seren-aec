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

#define CCMRAM __attribute__((section(".ccmram")))

#define CPU_SPEED		168

#define SPK_BUFFERS_NUM		4
#define SPK_CHANNELS		2
#define SPK_FREQUENCY		16000
#define SPK_FRAME_SIZE		(SPK_CHANNELS*SPK_FREQUENCY/100)  // assuming 10ms frame size

#define MIC_GAIN		2
#define MIC_PDM_FREQUENCY	1024000                        // PDM freq: MP45DT02 freq range: 1.0-3.25 Mhz
#define MIC_PDM_FRAME_SIZE_BITS	(MIC_PDM_FREQUENCY / 100)      // 10ms frame size in bits
#define MIC_PDM_FRAME_SIZE	(MIC_PDM_FRAME_SIZE_BITS / 16) // 10ms frame size in words
#define MIC_FREQUENCY           16000

#define UDP_PACKET_SEREN_AUDIO_HEADER_LEN	6
#define NICKLEN                         	16
#define UDP_PACKET_MASK_FTYPE           	0x0FFF

#define UDP_PACKET_FAMILY_HANDSHAKE     	0x0000
#define UDP_PACKET_TYPE_CALL            	0
#define UDP_PACKET_FTYPE_CALL           	(UDP_PACKET_FAMILY_HANDSHAKE | UDP_PACKET_TYPE_CALL)
#define UDP_PACKET_TYPE_TABLE           	3
#define UDP_PACKET_FTYPE_TABLE          	(UDP_PACKET_FAMILY_HANDSHAKE | UDP_PACKET_TYPE_TABLE)
#define UDP_PACKET_TYPE_BYE             	4
#define UDP_PACKET_FTYPE_BYE            	(UDP_PACKET_FAMILY_HANDSHAKE | UDP_PACKET_TYPE_BYE)

#define UDP_PACKET_FAMILY_DATA          	0x0100
#define UDP_PACKET_TYPE_AUDIO           	0
#define UDP_PACKET_FTYPE_AUDIO          	(UDP_PACKET_FAMILY_DATA | UDP_PACKET_TYPE_AUDIO)
#define UDP_PACKET_TYPE_NOP			1
#define UDP_PACKET_FTYPE_NOP			(UDP_PACKET_FAMILY_DATA | UDP_PACKET_TYPE_NOP)

#define UDP_PACKET_FAMILY_RTT			0x0400
#define UDP_PACKET_TYPE_RTTREQ			0
#define UDP_PACKET_FTYPE_RTTREQ			(UDP_PACKET_FAMILY_RTT | UDP_PACKET_TYPE_RTTREQ)
#define UDP_PACKET_TYPE_RTTANS			1
#define UDP_PACKET_FTYPE_RTTANS			(UDP_PACKET_FAMILY_RTT | UDP_PACKET_TYPE_RTTANS)

#define UDP_PACKET_FAMILY_TELEMETRY		0x0300
#define UDP_PACKET_TYPE_PLINFO			0
#define UDP_PACKET_FTYPE_PLINFO			(UDP_PACKET_FAMILY_TELEMETRY | UDP_PACKET_TYPE_PLINFO)
