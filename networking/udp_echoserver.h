#ifndef __ECHO_H__
#define __ECHO_H__

void udp_echoserver_init(uint16_t port);
void send_udp_message(struct udp_pcb* pcb, uint8_t* data, int dataLen);

#endif
