#ifndef UART_H
#define	UART_H

char uart_busy(void);
void uart_close(void);
//void ConfigIntUART1(unsigned int config);
char uart_available(void);
//unsigned int getsUART1(unsigned int length, unsigned int *buffer, unsigned int uart_data_wait);
//void OpenUART1(unsigned int config1, unsigned int config2, unsigned int ubrg);
//void putsUART1(unsigned int *buffer);
unsigned int uart_read(void);
void uart_write(unsigned int data);
unsigned char uart_getch();
void uart_putch(unsigned int c);
int uart_getch16();
void uart_putch16(int data);
void uart_flush();

#endif	/* UART_H */
