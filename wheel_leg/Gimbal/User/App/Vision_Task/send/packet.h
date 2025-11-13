#ifndef _PACKET_H
#define _PACKET_H

void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len);

void rm_dequeue_send_data(void* buf,uint16_t len);

#endif
