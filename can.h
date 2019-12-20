#ifndef _CAN_H_INCLUDED
#define _CAN_H_INCLUDED

#define RX_BUFFER_SIZE	50

void CAN_init(unsigned int);
void CAN_write(unsigned char *, unsigned int, unsigned char);
void CAN_read(unsigned char *);
unsigned char CAN_checkRX(void);
void CAN_getLastMessage(unsigned char *);

#endif
