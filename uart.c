#include    "uart.h"
#include    <p33FJ128MC802.h>

/**
 * @brief returns tstatus whethever transmission is in progress or bot
 * 
 * @return char info whether transmission is in progress
 */
char uart_busy(void)
{  
    return(!U1STAbits.TRMT);
} // end of uart_busy(...)

/**
 * @brief disabled the UART and cleares the interrupt enable & flag bits
 * 
 */
void uart_close(void)
{  
    U1MODEbits.UARTEN = 0;
	
	IEC0bits.U1RXIE = 0;
	IEC0bits.U1TXIE = 0;
	
	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;

} // end of uart_close(...)

/**
 * @brief set the priority for RX,TX interrupt and enable/disabled the interru[t]
 * 
 * @param config config enable/disable and priority
 */
/*void ConfigIntUART1(unsigned int config)
{
	// clear IF flags 
    IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;

	// set priority 
    IPC2bits.U1RXIP = 0x0007 & config;
    IPC3bits.U1TXIP = (0x0070 & config) >> 4; //OVDE JE PRE BILO IPC2

	// enable/disable interrupt 
    IEC0bits.U1RXIE = (0x0008 & config) >> 3;
    IEC0bits.U1TXIE = (0x0080 & config) >> 7;

} // end of ConfigIntUART1(...) */

/**
 * @brief checks whether there is any data that can be read from the input buffer
 * 
 * @return char if any data available in buffer
 */
char uart_available(void)
{
    return(U1STAbits.URXDA);
} // end of uart_available(...)

/**
 * @brief gets a string of data of specificed length if avaiable and buffers it into the specified buffer
 * 
 * @param length the length expected
 * @param buffer the recieved data to recorded to this array
 * @param uart_data_wait timeout value
 * @return unsigned int ndata bytes yet to be receieved
 */
 /*
unsigned int getsUART1(unsigned int length,unsigned int *buffer,
                       unsigned int uart_data_wait)
{
    int wait = 0;
	char *temp_ptr = (char *) buffer;
    while(length)                         // read till length is 0 
    {
        while(!uart_available())
        {
            if(wait < uart_data_wait)
                wait++ ;                  // ait for more data 
            else
                return(length);           // Time out- Return words/bytes to be read 
        }
        wait=0;
		if(U1MODEbits.PDSEL == 3)         // check if TX/RX is 8bits or 9bits 
		{
        	*buffer++ = U1RXREG;          // data word from HW buffer to SW buffer 
		}
		else
		{
	        *temp_ptr++ = U1RXREG & 0xFF; // data byte from HW buffer to SW buffer 
		}

        length--;
    }
    return(length);                       // number of data yet to be received i.e.,0 
} // end of getsUART1(...) */

/**
 * @brief configres the uart mode, interrupt modes and baud rate
 * 
 * @param config1 operation setting
 * @param config2 tx&rx interrupt modes
 * @param ubrg baud rate setting
 */
/*
void OpenUART1(unsigned int config1,unsigned int config2, unsigned int ubrg)
{
    U1BRG  = ubrg;     //
    U1MODE = config1;  //
    U1STA = config2;   //
} // end of OpenUART1 */

/**
 * @brief puts the data string to be transmitted int the transmit buffer (till NULL character)
 * 
 * @param buffer address of the string buffer to be transmitted
 */
/*
void putsUART1(unsigned int *buffer)
{
	char * temp_ptr = (char *) buffer;

    // transmit till NULL character is encountered 
    while((*buffer != '\0') && (*temp_ptr != '\0')) 
    {
        while(U1STAbits.UTXBF);     // wait if the buffer is full 

		if(U1MODEbits.PDSEL == 3)   // check if TX is 8bits or 9bits 
 		{
	        U1TXREG = *buffer++;    // transfer data word to TX reg 
		}
		else 
		{
			U1TXREG = *temp_ptr++;	// transfer data byte to TX reg 
    	}
	}
} // end of putsUART1*/

/**
 * @brief returns the contents of UxRXREG buffer
 * 
 * @return unsigned int value from the buffer
 */
unsigned int uart_read(void)
{
    if(U1MODEbits.PDSEL == 3)
		return (U1RXREG);
	else
		return (U1RXREG & 0xFF);
} // end of uart_read

/**
 * @brief writes data into the UxTXREG
 * 
 * @param data data to be written
 */
void uart_write(unsigned int data)
{
    if(U1MODEbits.PDSEL == 3)
        U1TXREG = data;
    else
        U1TXREG = data & 0xFF;
} // end of uart_write(...)

/**
 * @brief waits for 8bit data to be received
 * 
 * @return unsigned char returns the data it got
 */
unsigned char uart_getch()
{
    //U1STAbits.OERR=0;		
    while (!uart_available());	
    return (unsigned char)(uart_read() & 0x00ff);
} // end of uart_getch(...)

/**
 * @brief with the help of uart_getch, receive 16bit value
 * 
 * @return int the 16bit value
 */
int uart_getch16() {
    int data;
    data = (uart_getch() << 8) | uart_getch();
    return data;
} // end of uart_getch16(...)

/**
 * @brief helper function to write out data
 * 
 * @param c data we write out
 */
void uart_putch(unsigned int c)
{
    while(uart_busy());
    uart_write(c);
} // end of uart_putch(...)

/**
 * @brief same at uart_putch, but with 16bits
 * 
 * @param data 16bits we are writing
 */
void uart_putch16(int data) {
    uart_write(data >> 8);
    uart_write(data & 0xFF);
} // end of uart_putch16(...)

/**
 * @brief flush buffer
 * 
 */
void uart_flush() {
    SRbits.IPL = 7;
    SRbits.IPL = 0;
} // end of uart_flush(...)