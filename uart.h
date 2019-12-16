#ifndef UART_H
#define	UART_H

char BusyUART1(void);
void CloseUART1(void);
void ConfigIntUART1(unsigned int config);
char DataRdyUART1(void);
unsigned int getsUART1(unsigned int length, unsigned int *buffer, unsigned int uart_data_wait);
void OpenUART1(unsigned int config1, unsigned int config2, unsigned int ubrg);
void putsUART1(unsigned int *buffer);
unsigned int ReadUART1(void);
void WriteUART1(unsigned int data);
unsigned char getch();
void putch(unsigned int c);
int getch_16bit();
void putch_16bit(int data);

#endif	/* UART_H */
