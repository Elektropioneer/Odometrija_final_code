#include "motor.h"

/**
 * @brief update the left motor PWM channel
 * 
 * @param PWM 0-255 value to update the pwm to
 */
void motor_leftPWM(unsigned int PWM)
{
    P1DC1 = PWM;
} // end of motor_leftPWM(...)

/**
 * @brief update the right motor PWM channel
 * 
 * @param PWM 0-255 value to update the pwm to
 */
void motor_rightPWM(unsigned int PWM)
{
    P2DC1 = PWM;
} // end of motor_rightPWM(...)