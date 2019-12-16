#include    <p33FJ128MC802.h>

#ifndef MOTOR_H
#define	MOTOR_H

//makroi za zadavanje smera motora:
#define LEVI_NAPRED	LATBbits.LATB14=0; LATBbits.LATB12=1
#define LEVI_NAZAD	LATBbits.LATB14=1; LATBbits.LATB12=0
#define DESNI_NAPRED 	LATBbits.LATB11=1; LATBbits.LATB8=0
#define DESNI_NAZAD 	LATBbits.LATB11=0; LATBbits.LATB8=1

void LeviPWM(unsigned int PWM);
void DesniPWM(unsigned int PWM);

#endif	/* XC_HEADER_TEMPLATE_H */

