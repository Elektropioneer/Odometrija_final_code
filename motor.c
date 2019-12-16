#include "motor.h"

/**
 * @brief update the left motor PWM channel
 * 
 * @param PWM 0-255 value to update the pwm to
 */
void LeviPWM(unsigned int PWM)
{
    P1DC1 = PWM;
} // end of LeviPWM(...)

/**
 * @brief update the right motor PWM channel
 * 
 * @param PWM 0-255 value to update the pwm to
 */
void DesniPWM(unsigned int PWM)
{
    P2DC1 = PWM;
} // end of DesniPWM(...)