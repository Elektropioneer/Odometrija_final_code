#include    "uart.h"
#include    <p33FJ128MC802.h>

/**
 * @brief returns tstatus whethever transmission is in progress or bot
 * 
 * @return char info whether transmission is in progress
 */
char BusyUART1(void)
{  
    return(!U1STAbits.TRMT);
} // end of BusyUART1(...)

/**
 * @brief disabled the UART and cleares the interrupt enable & flag bits
 * 
 */
void CloseUART1(void)
{  
    U1MODEbits.UARTEN = 0;
	
	IEC0bits.U1RXIE = 0;
	IEC0bits.U1TXIE = 0;
	
	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;

} // end of CloseUART1(...)

/*********************************************************************
* Function Name     : ConfigIntUART1
* Description       : This function sets priority for RX,TX interrupt  
*                     and enable/disables the interrupt
* Parameters        : unsigned int config enable/disable and priority
* Return Value      : None
*********************************************************************/

/**
 * @brief set the priority for RX,TX interrupt and enable/disabled the interru[t]
 * 
 * @param config config enable/disable and priority
 */
void ConfigIntUART1(unsigned int config)
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

} // end of ConfigIntUART1(...)

/**
 * @brief checks whether there is any data that can be read from the input buffer
 * 
 * @return char if any data available in buffer
 */
char DataRdyUART1(void)
{
    return(U1STAbits.URXDA);
} // end of DataRdyUART1(...)

/**
 * @brief gets a string of data of specificed length if avaiable and buffers it into the specified buffer
 * 
 * @param length the length expected
 * @param buffer the recieved data to recorded to this array
 * @param uart_data_wait timeout value
 * @return unsigned int ndata bytes yet to be receieved
 */
unsigned int getsUART1(unsigned int length,unsigned int *buffer,
                       unsigned int uart_data_wait)
{
    int wait = 0;
	char *temp_ptr = (char *) buffer;
    while(length)                         // read till length is 0 
    {
        while(!DataRdyUART1())
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
} // end of getsUART1(...)

/**
 * @brief configres the uart mode, interrupt modes and baud rate
 * 
 * @param config1 operation setting
 * @param config2 tx&rx interrupt modes
 * @param ubrg baud rate setting
 */
void OpenUART1(unsigned int config1,unsigned int config2, unsigned int ubrg)
{
    U1BRG  = ubrg;     /* baud rate */
    U1MODE = config1;  /* operation settings */
    U1STA = config2;   /* TX & RX interrupt modes */
} // end of OpenUART1

/**
 * @brief puts the data string to be transmitted int the transmit buffer (till NULL character)
 * 
 * @param buffer address of the string buffer to be transmitted
 */
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
} // end of putsUART1

/**
 * @brief returns the contents of UxRXREG buffer
 * 
 * @return unsigned int value from the buffer
 */
unsigned int ReadUART1(void)
{
    if(U1MODEbits.PDSEL == 3)
		return (U1RXREG);
	else
		return (U1RXREG & 0xFF);
} // end of ReadUART1

/**
 * @brief writes data into the UxTXREG
 * 
 * @param data data to be written
 */
void WriteUART1(unsigned int data)
{
    if(U1MODEbits.PDSEL == 3)
        U1TXREG = data;
    else
        U1TXREG = data & 0xFF;
} // end of WriteUART1(...)

/**
 * @brief waits for 8bit data to be received
 * 
 * @return unsigned char returns the data it got
 */
unsigned char getch()
{
    //U1STAbits.OERR=0;		
    while (!DataRdyUART1());	
    return (unsigned char)(ReadUART1() & 0x00ff);
} // end of getch(...)

/**
 * @brief with the help of getch, receive 16bit value
 * 
 * @return int the 16bit value
 */
int getch_16bit() {
    int data;
    data = (getch() << 8) | getch();
    return data;
} // end of getch_16bit(...)

/**
 * @brief helper function to write out data
 * 
 * @param c data we write out
 */
void putch(unsigned int c)
{
    while(BusyUART1());
    WriteUART1(c);
} // end of putch(...)

/**
 * @brief same at putch, but with 16bits
 * 
 * @param data 16bits we are writing
 */
void putch_16bit(int data) {
    WriteUART1(data >> 8);
    WriteUART1(data & 0xFF);
} // end of putch_16bit(...)

/**
 * @brief flush buffer
 * 
 */
void FlushUART1() {
    SRbits.IPL = 7;
    SRbits.IPL = 0;
} // end of FlushUART1(...)