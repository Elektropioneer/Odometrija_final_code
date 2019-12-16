#include	<p33FJ128MC802.h>
#include    <libpic30.h>
#include    <xc.h>
#include	"uart.h"

#define PIN_UART1_RX    0           // UART1 RX na RP0- pin 4
#define PIN_UART1_TX    3           // UART1 TX na RP1- pin 5
#define PIN_QEA1        2           // QEI1A na RP2
#define PIN_QEB1        3           // QEI1B na RP3
#define PIN_QEA2        4           // QEI2A na RP4
#define PIN_QEB2        7           // QEI2B na RP7

void UARTinit();
void CloseMCPWM(void);
void PWMinit();
void TimerInit();
void QEIinit();
void PortInit();
void PinsInit();
void OscillatorInit();