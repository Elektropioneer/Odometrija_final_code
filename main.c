#define FCY	30000000ULL
#include <libpic30.h>
#include <xc.h>
#include "init.h"
#include "sinus.h"
#include "kretanje.h"
#include "globals.h"
#include "uart.h"
#include "can.h"


_FOSCSEL(FNOSC_PRI);	// primary oscillator without PLL
// use on >10 MHz; OSCIOFNC_ON assign pin; FCKSM_CSECMD clock switch enabled and clock monitoring disabled
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_XT & IOL1WAY_OFF); 
_FWDT(FWDTEN_OFF);      // kill watchdog
_FPOR(PWMPIN_ON &/* & BOREN_OFF & */HPOL_ON & LPOL_ON & FPWRT_PWR2 & ALTI2C_ON); // assign pins

void LeviPWM(unsigned int PWM);
void DesniPWM(unsigned int PWM);

unsigned char rxData[8];
unsigned char txData[8];

//POMOCNE PROMENLJIVE:
char brint = 0;
long positionR, positionL;              //trenutne pozicije na enkoderima
int vR, vL, greska_pred, greska_pred_R; //trenutne brzine na motorima
int zaglavL, zaglavR;
unsigned char brzinaL;
long L=0, orientation=0, teta=0;
long long int Xlong=0, Ylong=0;
float vmax, accel;
float omega, alfa;
long X=0, Y=0;
long brojac,i;
unsigned long sys_time = 0;
//PROMENLJIVE POTREBNE ZA REGULACIJU
int PWML, PWMD;
long t_ref=0, d_ref=0;
float v_ref;


/**
 * @brief T1Interrupt - main PID loop - 1ms loop
 * 
 */
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
    long long d, t;
    long long sint, cost;
    long long x, y;
    long greska;
    int brzina, commande_distance, commande_rotation;

    // keep track of system time
    sys_time++;

    // execute PID on every 5ms
    // TODO: move the 5 into a variable in config.h
    if (++brint == 5)
    {
        brint = 0;

        vR = -(int)POS2CNT;     // read the increments from the encoder
        POS2CNT = 0;		    // reseting the counter
        //positionR += (1.051335 * (double)vR);//1.05131	//azuriramo trenutnu poziciju
        positionR += vR;        // calculate the distance (encoder) it moved in the last 5ms
        
        vL = (int)POS1CNT;	    // read the increments from the encoder
        POS1CNT = 0;		    // reseting the counter
        positionL += vL;	    // calculate the distance (encoder) it moved in the last 5ms
		
        L = (positionR + positionL) / 2;              // calculate the robots distance
        orientation = (positionR - positionL) % K1;   // the angle it moved at
        
        // depending on the orientation we set it in positive/negative
        if(orientation > K1/2)
            orientation -= K1;
        if(orientation < -K1/2)
            orientation += K1;
        
        // calculating the theta angle from the orientation
        teta = (orientation * 16384) / (K1 / 2);
		
        // if the theta is in - add 32768 to even it out
        if (teta < 0)
        	teta += 32768;

        // add together the distance (encoder increments) - will be divided later
        d = vR + vL;

        // based on the theta calculate theta, sin and cos
        if(teta < 8192) {
            t = teta;
            sint = sinus[t];
            cost = sinus[8191 - t];
        } else {
            if (teta < 16384) {
                t = teta - 8192;
                sint = sinus[8191 - t];
                cost = -sinus[t];
            } else {
                if (teta < 24576) {
                    t = teta - 16384;
                    sint = -sinus[t];
                    cost = -sinus[8191 - t];
                } else {
                    t = teta - 24576;
                    sint = -sinus[8191 - t];
                    cost = sinus[t];
                }
            }
        }

        // through the distance the encoders moved and with the help of cos and sin we can determine our x & y movement
        x = d * cost;
        y = d * sint;

        // global X and Y position (increments)
        Xlong += x;
        Ylong += y;

        // converting increments into millimeters
        X = ((long long)Xlong) >> 16;
        X /= K2;
        Y = ((long long)Ylong) >> 16;
        Y /= K2;

        // PID regulator - distance regulator
        brzina = (vL + vR) / 2;                         // get the speed based on the increments it got 
        greska = d_ref - L;                             // d_ref -> where we want it to be; L -> where we are                           
        zaglavL = greska >= 0 ? greska : -greska;       // stuck variable, checks if we totally stuck and fucked

        // with the help of PD we calculate the PWM speed for distance (forward/backward)
        commande_distance = greska * Gp_D - Gd_D * brzina;

        // PID regulator - rotation regulator
        brzina = vL - vR;                               // get the speed based on the increments it got

        // calculate the error between where we are and where we want to be
        greska = (orientation - t_ref) % K1;            // orientation -> where we are; t_ref -> where we want to be;
        
        // the error depends if it's positive/negative
        if (greska > K1/2) {
            greska -= K1;
        }
        if (greska < -K1/2) {
            greska += K1;
        }

        // check the stuck condition
        zaglavR = greska >= 0 ? greska : -greska;
        
        // with the help of PD we calculate the PWM speed for distance (rotation)
        commande_rotation = greska * Gp_T - brzina * Gd_T;

        // calculate the summed PWM for the motors 
        // for left it's minus the rotation because one motors turn the other way
        PWML = commande_distance - commande_rotation;
        PWMD = commande_distance + commande_rotation;
        
        // motor saturation (max/min)
        if(PWMD <= -3200)
            PWMD = -3200;
        else if(PWMD >= 3200)
            PWMD = 3200;
        if(PWML <= -3200)
            PWML = -3200;
        else if(PWML >= 3200)
            PWML = 3200;
		
        // depending on the PWM value, send the motor forward/backward
        // left motor
        if(PWML >= 0) {
                LEVI_NAPRED;
                LeviPWM(PWML);
        } else {
                LEVI_NAZAD;
                LeviPWM(-PWML);
        }

        // right motor
        if (PWMD >= 0) {
                DESNI_NAPRED;
                DesniPWM(PWMD);
        } else {
                DESNI_NAZAD;
                DesniPWM(-PWMD);
        }
    } // end of regulation

    // clear interrupt flag
    IFS0bits.T1IF = 0;    
} // end of interrupt

/**
 * @brief reset the regulation parameters
 * 
 */
void ResetDriver()
{
    positionR = positionL=0;        // encoder sum position
    L = orientation = 0;            // distance/orientation
    vR = vL = 0;                    // encoder position
} // end of ResetDriver(...)

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

static void uart_flush() {
    SRbits.IPL = 7;
    SRbits.IPL = 0;
}

int main(void)
{
	 
    OscillatorInit();
    PinsInit();    
    
    // init CAN BUS
    //CAN_init(DRIVER_IDENTIFICATOR); 

    int tmp;
    char komanda, v, smer;
    int Xc, Yc, ugao;

    PortInit();
    UARTinit();
    TimerInit();
    QEIinit();
    PWMinit();

    resetDriver();

    setSpeed(0x80);
    setSpeedAccel(K2);	//K2 je za 1m/s /bilo je 2
    
    int tmpX, tmpY, tmpO;               // vrednosti X,Y, orientation koji dobijes preko komunikacije
    unsigned char rxBuffer[8];          // buffer gde stavis sve   

    // podesis maximalni speed (najbolje je kad ukljucis robota da to uradis preko glavne)
    setSpeed(70);
    
    
    while(1)
    {
        // if data has arrived
        if(DataRdyUART1() > 0) {
            
            // check the first byte
            komanda = getch();
            
            // select the function depending on the first byte
            switch(komanda)
            {
                // set position
                case 'I':
                    tmpX = getch_16bit();
                    tmpY = getch_16bit();
                    tmpO = getch_16bit();
                    
                    
                    setPosition(tmpX, tmpY, tmpO);
                    break;

                // send back status and position
                case 'P':
                    sendStatusAndPosition();
                    break;

                // set the max speed
                case 'V':
                    tmp = getch();
                    setSpeed(tmp);

                    break;

                // move robot forward/backward
                case 'D':
                    tmp = getch_16bit();
                    v = getch();            // maximal speed
                    PWMinit();              // ako je predhodno bio soft stop da opet inituje
                    kretanje_pravo(tmp, v);

                    break;

                // relative angle
                case 'T':
                    tmp = getch_16bit();
                    PWMinit();
                    okret(tmp);

                    break;

                // absolute angle
                case 'A':
                    tmp = getch_16bit();
                    PWMinit();
                    apsolutni_ugao(tmp);

                    break;

                // goto xy
                case 'G':
                    tmpX = getch_16bit();
                    tmpY = getch_16bit();
                    v = getch();
                    smer = getch();
                    PWMinit();
                    gotoXY(tmpX, tmpY, v, smer);

                    break;

                // kurva
                case 'Q':
                    tmpX = getch_16bit();
                    tmpY = getch_16bit();
                    tmpO = getch_16bit();
                    smer = getch();
                    PWMinit();
                    kurva(tmpX, tmpY, tmpO, smer);

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
