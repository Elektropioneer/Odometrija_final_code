#include "motor.h"

//#define ESC_MIN 1000
//#define ESC_MAX 2000

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
    //P2DC1 = 100; // 300us
    //P2DC1 = 15000; // 4000us
    //P2DC1 = 10000; // 9200
    //P2DC1 = 5000; // 5200
    //P2DC1 = 2500; // 2600
    //P2DC1 = 1800; // 1500
    //P2DC1 = 1000; // 1000
    //P2DC1 = 800; // 840
 */

/**
 * @brief update the left motor PWM channel
 * 
 * @param PWM 0-255 value to update the pwm to
 */
void motor_leftPWM(unsigned int PWM)
{
    P1DC1 = (uint16_t) map(PWM, 0, 255, ESC_MIN, ESC_MAX);
} // end of motor_leftPWM(...)

/**
 * @brief update the right motor PWM channel
 * 
 * @param PWM 0-255 value to update the pwm to
 */
void motor_rightPWM(unsigned int PWM)
{
    P2DC1 = (uint16_t) map(PWM, 0, 255, ESC_MIN, ESC_MAX);
} // end of motor_rightPWM(...)