#define FCY	30000000ULL
#include <libpic30.h>
#include <xc.h>
#include "init.h"
#include "sinus.h"
#include "movement.h"
#include "globals.h"
#include "uart.h"
#include "can.h"
#include "motor.h"


_FOSCSEL(FNOSC_PRI);	// primary oscillator without PLL
// use on >10 MHz; OSCIOFNC_ON assign pin; FCKSM_CSECMD clock switch enabled and clock monitoring disabled
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_XT & IOL1WAY_OFF); 
_FWDT(FWDTEN_OFF);      // kill watchdog
_FPOR(PWMPIN_ON &/* & BOREN_OFF & */HPOL_ON & LPOL_ON & FPWRT_PWR2 & ALTI2C_ON); // assign pins

//POMOCNE PROMENLJIVE:
char odometry_loop_counter = 0;
long encoder_rightIncrements, encoder_leftIncrements;              //trenutne pozicije na enkoderima
int encoder_rightCurrentIncrements, encoder_leftCurrentIncrements, greska_pred, greska_pred_R; //trenutne brzine na motorima
int odometry_stuckOrientation, odometry_stuckDistance;
unsigned char odometry_maxSpeedSet;
long odometry_incrementsDistance=0, odometry_incrementsOrientation=0, odometry_orientationTeta=0;
long long int odometry_incrementsX=0, odometry_incrementsY=0;
float odometry_speedMax, odometry_acceleration;
float odometry_speedOmega, odometry_accelerationAlpha;
long odometry_milliX=0, odometry_milliY=0;
unsigned long sys_time = 0;
//PROMENLJIVE POTREBNE ZA REGULACIJU
int motor_currentLeftPWM, motor_currentRightPWM;
long odometry_refrenceOrientation=0, odometry_refrenceDistance=0;
float odometry_refrenceSpeed;


/**
 * @brief T1Interrupt - main PID loop - 1ms loop
 * 
 */
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
    long long odometry_incrementsCurrentDistance, odometry_orientationArea;
    long long odometry_sint, odometry_cost;
    long odometry_regulatorError;
    int odometry_currentIncrementSpeed, odometry_regulatorDistance, odometry_regulatorRotation;

    // keep track of system time
    sys_time++;

    // execute PID on every 5ms
    // TODO: move the 5 into a variable in config.h
    if (++odometry_loop_counter == 5)
    //if(something == 1)
    {
        // reset counter
        odometry_loop_counter = 0;

        encoder_rightCurrentIncrements = -(int)POS2CNT;                 // read the increments from the encoder
        POS2CNT = 0;		                                            // reseting the counter

        //encoder_rightIncrements += (1.051335 * (double)encoder_rightCurrentIncrements);//1.05131	//azuriramo trenutnu poziciju
        encoder_rightIncrements += encoder_rightCurrentIncrements;        // calculate the distance (encoder) it moved in the last 5ms
        
        encoder_leftCurrentIncrements = (int)POS1CNT;	                // read the increments from the encoder
        POS1CNT = 0;		                                            // reseting the counter
        encoder_leftIncrements += encoder_leftCurrentIncrements;	    // calculate the distance (encoder) it moved in the last 5ms
		
        odometry_incrementsDistance     = (encoder_rightIncrements + encoder_leftIncrements) / 2;               // calculate the robots distance
        odometry_incrementsOrientation  = (encoder_rightIncrements - encoder_leftIncrements) % K1;           // the angle it moved at
        
        // depending on the odometry_incrementsOrientation we set it in positive/negative
        if(odometry_incrementsOrientation > K1/2)
            odometry_incrementsOrientation -= K1;
        if(odometry_incrementsOrientation < -K1/2)
            odometry_incrementsOrientation += K1;
        
        // calculating the theta angle from the odometry_incrementsOrientation
        odometry_orientationTeta = (odometry_incrementsOrientation * 16384) / (K1 / 2);
		
        // if the theta is in - add 32768 to even it out
        if (odometry_orientationTeta < 0)
        	odometry_orientationTeta += 32768;

        // add together the distance (encoder increments) - will be divided later
        odometry_incrementsCurrentDistance = encoder_rightCurrentIncrements + encoder_leftCurrentIncrements;

        // based on the theta calculate theta, sin and cos (depending on the theta get the sinus in one of the four areas)
        if(odometry_orientationTeta < 8192) {
            odometry_orientationArea = odometry_orientationTeta;
            odometry_sint = sinus[odometry_orientationArea];
            odometry_cost = sinus[8191 - odometry_orientationArea];
        } else {
            if (odometry_orientationTeta < 16384) {
                odometry_orientationArea = odometry_orientationTeta - 8192;
                odometry_sint = sinus[8191 - odometry_orientationArea];
                odometry_cost = -sinus[odometry_orientationArea];
            } else {
                if (odometry_orientationTeta < 24576) {
                    odometry_orientationArea = odometry_orientationTeta - 16384;
                    odometry_sint = -sinus[odometry_orientationArea];
                    odometry_cost = -sinus[8191 - odometry_orientationArea];
                } else {
                    odometry_orientationArea = odometry_orientationTeta - 24576;
                    odometry_sint = -sinus[8191 - odometry_orientationArea];
                    odometry_cost = sinus[odometry_orientationArea];
                }
            }
        }

        // through the distance the encoders moved and with the help of cos and sin we can determine our x & y movement
        // global odometry_milliX and odometry_milliY position (increments)
        odometry_incrementsX += (odometry_incrementsCurrentDistance * odometry_cost);
        odometry_incrementsY += (odometry_incrementsCurrentDistance * odometry_sint);

        // converting increments into millimeters
        odometry_milliX = ((long long)odometry_incrementsX) >> 16;
        odometry_milliX /= K2;
        odometry_milliY = ((long long)odometry_incrementsY) >> 16;
        odometry_milliY /= K2;

        // PID regulator - distance regulator
        odometry_currentIncrementSpeed = (encoder_leftCurrentIncrements + encoder_rightCurrentIncrements) / 2;                         // get the speed based on the increments it got 
        odometry_regulatorError = odometry_refrenceDistance - odometry_incrementsDistance;                             // odometry_refrenceDistance -> where we want it to be; odometry_incrementsDistance -> where we are                           
        odometry_stuckDistance = encoder_leftCurrentIncrements = odometry_regulatorError >= 0 ? odometry_regulatorError : -odometry_regulatorError;       // stuck variable, checks if we totally stuck and fucked

        // with the help of PD we calculate the PWM speed for distance (forward/backward)
        odometry_regulatorDistance = odometry_regulatorError * regulator_distanceP - regulator_distanceD * odometry_currentIncrementSpeed;

        // PID regulator - rotation regulator
        odometry_currentIncrementSpeed = encoder_leftCurrentIncrements - encoder_rightCurrentIncrements;                               // get the speed based on the increments it got

        // calculate the error between where we are and where we want to be
        odometry_regulatorError = (odometry_incrementsOrientation - odometry_refrenceOrientation) % K1;            // odometry_incrementsOrientation -> where we are; odometry_refrenceOrientation -> where we want to be;
        
        // the error depends if it's positive/negative
        if (odometry_regulatorError > K1/2) {
            odometry_regulatorError -= K1;
        }
        if (odometry_regulatorError < -K1/2) {
            odometry_regulatorError += K1;
        }

        // check the stuck condition
        odometry_stuckOrientation = encoder_rightCurrentIncrements = odometry_regulatorError >= 0 ? odometry_regulatorError : -odometry_regulatorError;
        
        // with the help of PD we calculate the PWM speed for distance (rotation)
        odometry_regulatorRotation = odometry_regulatorError * regulator_rotationP - odometry_currentIncrementSpeed * regulator_rotationD;

        // calculate the summed PWM for the motors 
        // for left it's minus the rotation because one motors turn the other way
        motor_currentLeftPWM = odometry_regulatorDistance - odometry_regulatorRotation;
        motor_currentRightPWM = odometry_regulatorDistance + odometry_regulatorRotation;

        // when setting up PID uncomment for forward/backward
        //motor_currentLeftPWM = odometry_regulatorDistance;
        //motor_currentRightPWM = odometry_regulatorDistance;

        // when setting up PID uncomment for rotation
        //motor_currentLeftPWM = -odometry_regulatorRotation;
        //motor_currentRightPWM = odometry_regulatorRotation;
        
        // motor saturation (max/min)
        if(motor_currentRightPWM <= -(motor_saturationPWMValue)) {
            motor_currentRightPWM = -(motor_saturationPWMValue);
        } else if(motor_currentRightPWM >= motor_saturationPWMValue) {
            motor_currentRightPWM = motor_saturationPWMValue;
        }
        
        if(motor_currentLeftPWM <= -(motor_saturationPWMValue)) {
            motor_currentLeftPWM = -(motor_saturationPWMValue);
        } else if(motor_currentLeftPWM >= motor_saturationPWMValue) {
            motor_currentLeftPWM = motor_saturationPWMValue;
        }
		
        // depending on the PWM value, send the motor forward/backward
        // left motor
        if(motor_currentLeftPWM >= 0) {
                motor_leftForward;
                motor_leftPWM(motor_currentLeftPWM);
        } else {
                motor_leftBackward;
                motor_leftPWM(-motor_currentLeftPWM);
        }

        // right motor
        if (motor_currentRightPWM >= 0) {
                motor_rightForward;
                motor_rightPWM(motor_currentRightPWM);
        } else {
                motor_rightBackward;
                motor_rightPWM(-motor_currentRightPWM);
        }
    } // end of regulation

    // clear interrupt flag
    IFS0bits.T1IF = 0;    
} // end of interrupt12:30pm


int main(void)
{
	 
    OscillatorInit();
    PinsInit();    
    
    // init CAN BUS
    //CAN_init(DRIVER_IDENTIFICATOR); 

    PortInit();                 // init for other pins
    UARTinit();                 // init for communication
    TimerInit();                // init for interrupt timer
    PWMinit();                  // init for motors
    int i=0;
    //3.27675
    motor_rightPWM(255);
    __delay_ms(6000);
    motor_rightPWM(0);
    __delay_ms(5000);
    QEIinit();
    robot_resetDriver();

    robot_setSpeed(0x80);
    odometry_setAcceleration(K2);	

    // default max speed
    robot_setSpeed(70);
    
    int uart_intData1, uart_intData2, uart_intData3;
    unsigned char uart_command, uart_charData1, uart_charData2;
    
    while(1)
    {
        // if data has arrived
        if(uart_available() > 0) {
            
            // check the first byte
            uart_command = uart_getch();
            
            // select the function depending on the first byte
            switch(uart_command)
            {
                // set position
                case 'I':
                    uart_intData1 = uart_getch16();
                    uart_intData2 = uart_getch16();
                    uart_intData3 = uart_getch16();
                    
                    
                    robot_setPosition(uart_intData1, uart_intData2, uart_intData3);
                    break;

                // send back status and position
                case 'P':
                    robot_returnInfo();
                    break;

                // set the max speed
                case 'V':
                    uart_charData1 = uart_getch();
                    robot_setSpeed(uart_charData1);

                    break;

                // move robot forward/backward
                case 'D':
                    uart_intData1 = uart_getch16();
                    uart_charData1 = uart_getch();            
                    PWMinit();             
                    robot_moveLinear(uart_intData1, uart_charData1);

                    break;

                // relative angle
                case 'T':
                    uart_intData1 = uart_getch16();
                    PWMinit();
                    robot_rotate(uart_intData1);

                    break;

                // absolute angle
                case 'A':
                    uart_intData1 = uart_getch16();
                    PWMinit();
                    robot_rotateAbsolute(uart_intData1);

                    break;

                // goto xy
                case 'G':
                    uart_intData1 = uart_getch16();
                    uart_intData2 = uart_getch16();
                    uart_charData1 = uart_getch();
                    uart_charData2 = uart_getch();
                    PWMinit();
                    robot_moveXY(uart_intData1, uart_intData2, uart_charData1, uart_charData2);

                    break;

                // robot_arc
                case 'Q':
                    uart_intData1 = uart_getch16();
                    uart_intData2 = uart_getch16();
                    uart_intData3 = uart_getch16();
                    uart_charData1 = uart_getch();
                    PWMinit();
                    robot_arc(uart_intData1, uart_intData2, uart_intData3, uart_charData1);

                    break;

                // hard robot_stop
                case 'S':
                   
                    robot_stop();

                    break;

                // robot_stop and turn off PWM
                case 's':
                    
                    robot_stop();
                    CloseMCPWM();

                    break;

                // only turn off PWM
                case 'K':
                    
                    CloseMCPWM();
                    break;
                
                default:
                    robot_forceStatus(STATUS_ERROR);
                    break;
            }
        } 
    }

    return 0;
}
