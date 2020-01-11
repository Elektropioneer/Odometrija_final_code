#include    <p33FJ128MC802.h>

#ifndef MOTOR_H
#define	MOTOR_H


#define ESC_MIN 850
#define ESC_MAX 2000

//makroi za zadavanje smera motora:
#define motor_leftForward	    LATBbits.LATB14=0; LATBbits.LATB12=1
#define motor_leftBackward	    LATBbits.LATB14=1; LATBbits.LATB12=0
#define motor_rightForward 	    LATBbits.LATB11=1; LATBbits.LATB8=0
#define motor_rightBackward 	LATBbits.LATB11=0; LATBbits.LATB8=1

void motor_leftPWM(unsigned int PWM);
void motor_rightPWM(unsigned int PWM);

#endif

