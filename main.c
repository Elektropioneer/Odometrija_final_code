#define FCY	30000000ULL
#include <libpic30.h>
#include <xc.h>
#include "init.h"
#include "sinus.h"
#include "kretanje.h"
#include "globals.h"
#include "uart.h"
#include "can.h"
#include "motor.h"


_FOSCSEL(FNOSC_PRI);	// primary oscillator without PLL
// use on >10 MHz; OSCIOFNC_ON assign pin; FCKSM_CSECMD clock switch enabled and clock monitoring disabled
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_XT & IOL1WAY_OFF); 
_FWDT(FWDTEN_OFF);      // kill watchdog
_FPOR(PWMPIN_ON &/* & BOREN_OFF & */HPOL_ON & LPOL_ON & FPWRT_PWR2 & ALTI2C_ON); // assign pins

unsigned char rxData[8];
unsigned char txData[8];

//POMOCNE PROMENLJIVE:
char odometry_loop_counter = 0;
long encoder_rightIncrements, encoder_leftIncrements;              //trenutne pozicije na enkoderima
int encoder_rightCurrentIncrements, encoder_leftCurrentIncrements, greska_pred, greska_pred_R; //trenutne brzine na motorima
int odometry_stuckOrientation, odometry_stuckDistance;
unsigned char brzinaL;
long odometry_incrementsDistance=0, odometry_incrementsOrientation=0, odometry_orientationTeta=0;
long long int odometry_incrementsX=0, odometry_incrementsY=0;
float vmax, accel;
float omega, alfa;
long odometry_milliX=0, odometry_milliY=0;
long brojac,i;
unsigned long sys_time = 0;
//PROMENLJIVE POTREBNE ZA REGULACIJU
int motor_currentLeftPWM, motor_currentRightPWM;
long t_ref=0, d_ref=0;
float v_ref;


/**
 * @brief T1Interrupt - main PID loop - 1ms loop
 * 
 */
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
    long long odometry_incrementsCurrentDistance, t;
    long long odometry_sint, odometry_cost;
    long odometry_regulatorError;
    int odometry_currentIncrementSpeed, odometry_regulatorDistance, odometry_regulatorRotation;

    // keep track of system time
    sys_time++;

    // execute PID on every 5ms
    // TODO: move the 5 into a variable in config.h
    if (++odometry_loop_counter == 5)
    {
        odometry_loop_counter = 0;

        encoder_rightCurrentIncrements = -(int)POS2CNT;     // read the increments from the encoder
        POS2CNT = 0;		    // reseting the counter
        //encoder_rightIncrements += (1.051335 * (double)encoder_rightCurrentIncrements);//1.05131	//azuriramo trenutnu poziciju
        encoder_rightIncrements += encoder_rightCurrentIncrements;        // calculate the distance (encoder) it moved in the last 5ms
        
        encoder_leftCurrentIncrements = (int)POS1CNT;	    // read the increments from the encoder
        POS1CNT = 0;		    // reseting the counter
        encoder_leftIncrements += encoder_leftCurrentIncrements;	    // calculate the distance (encoder) it moved in the last 5ms
		
        odometry_incrementsDistance = (encoder_rightIncrements + encoder_leftIncrements) / 2;              // calculate the robots distance
        odometry_incrementsOrientation = (encoder_rightIncrements - encoder_leftIncrements) % K1;   // the angle it moved at
        
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

        // based on the theta calculate theta, sin and cos
        if(odometry_orientationTeta < 8192) {
            t = odometry_orientationTeta;
            odometry_sint = sinus[t];
            odometry_cost = sinus[8191 - t];
        } else {
            if (odometry_orientationTeta < 16384) {
                t = odometry_orientationTeta - 8192;
                odometry_sint = sinus[8191 - t];
                odometry_cost = -sinus[t];
            } else {
                if (odometry_orientationTeta < 24576) {
                    t = odometry_orientationTeta - 16384;
                    odometry_sint = -sinus[t];
                    odometry_cost = -sinus[8191 - t];
                } else {
                    t = odometry_orientationTeta - 24576;
                    odometry_sint = -sinus[8191 - t];
                    odometry_cost = sinus[t];
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
        odometry_regulatorError = d_ref - odometry_incrementsDistance;                             // d_ref -> where we want it to be; odometry_incrementsDistance -> where we are                           
        odometry_stuckDistance = encoder_leftCurrentIncrements = odometry_regulatorError >= 0 ? odometry_regulatorError : -odometry_regulatorError;       // stuck variable, checks if we totally stuck and fucked

        // with the help of PD we calculate the PWM speed for distance (forward/backward)
        odometry_regulatorDistance = odometry_regulatorError * Gp_D - Gd_D * odometry_currentIncrementSpeed;

        // PID regulator - rotation regulator
        odometry_currentIncrementSpeed = encoder_leftCurrentIncrements - encoder_rightCurrentIncrements;                               // get the speed based on the increments it got

        // calculate the error between where we are and where we want to be
        odometry_regulatorError = (odometry_incrementsOrientation - t_ref) % K1;            // odometry_incrementsOrientation -> where we are; t_ref -> where we want to be;
        
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
        odometry_regulatorRotation = odometry_regulatorError * Gp_T - odometry_currentIncrementSpeed * Gd_T;

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
        if(motor_currentRightPWM <= -(motor_saturationPWMValue))
            motor_currentRightPWM = -(motor_saturationPWMValue);
        else if(motor_currentRightPWM >= motor_saturationPWMValue)
            motor_currentRightPWM = motor_saturationPWMValue;
        if(motor_currentLeftPWM <= -(motor_saturationPWMValue))
            motor_currentLeftPWM = -(motor_saturationPWMValue);
        else if(motor_currentLeftPWM >= motor_saturationPWMValue)
            motor_currentLeftPWM = motor_saturationPWMValue;
		
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
    QEIinit();                  // init for encoders
    PWMinit();                  // init for motors

    resetDriver();

    setSpeed(0x80);
    setSpeedAccel(K2);	

    // default max speed
    setSpeed(70);
    
    int data_tmp, data_tmpX, data_tmpY, data_tmpO;
    unsigned char uart_command, data_tmpSpeed, data_tmpDirection;
    unsigned char uc_data_tmp;
    
    while(1)
    {
        // if data has arrived
        if(DataRdyUART1() > 0) {
            
            // check the first byte
            uart_command = getch();
            
            // select the function depending on the first byte
            switch(uart_command)
            {
                // set position
                case 'I':
                    data_tmpX = getch_16bit();
                    data_tmpY = getch_16bit();
                    data_tmpO = getch_16bit();
                    
                    
                    setPosition(data_tmpX, data_tmpY, data_tmpO);
                    break;

                // send back status and position
                case 'P':
                    sendStatusAndPosition();
                    break;

                // set the max speed
                case 'V':
                    uc_data_tmp = getch();
                    setSpeed(uc_data_tmp);

                    break;

                // move robot forward/backward
                case 'D':
                    data_tmp = getch_16bit();
                    data_tmpSpeed = getch();            
                    PWMinit();             
                    kretanje_pravo(data_tmp, data_tmpSpeed);

                    break;

                // relative angle
                case 'T':
                    data_tmp = getch_16bit();
                    PWMinit();
                    okret(data_tmp);

                    break;

                // absolute angle
                case 'A':
                    data_tmp = getch_16bit();
                    PWMinit();
                    apsolutni_ugao(data_tmp);

                    break;

                // goto xy
                case 'G':
                    data_tmpX = getch_16bit();
                    data_tmpY = getch_16bit();
                    data_tmpSpeed = getch();
                    data_tmpDirection = getch();
                    PWMinit();
                    gotoXY(data_tmpX, data_tmpY, data_tmpSpeed, data_tmpDirection);

                    break;

                // kurva
                case 'Q':
                    data_tmpX = getch_16bit();
                    data_tmpY = getch_16bit();
                    data_tmpO = getch_16bit();
                    data_tmpDirection = getch();
                    PWMinit();
                    kurva(data_tmpX, data_tmpY, data_tmpO, data_tmpDirection);

                    break;

                // hard stop
                case 'S':
                   
                    stop();

                    break;

                // stop and turn off PWM
                case 's':
                    
                    stop();
                    CloseMCPWM();

                    break;

                // only turn off PWM
                case 'K':
                    
                    CloseMCPWM();
                    break;
                
                default:
                    forceStatus(STATUS_ERROR);
                    break;
            }
        } 
    }

    return 0;
}
