#define FCY	30000000ULL
#include <libpic30.h>
#include <xc.h>
#include "init.h"
#include "sinus.h"
#include "kretanje.h"
#include "globals.h"
#include "uart.h"
#include "pwm.h"
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
}

void ResetDriver()
{
    //inicijalizacija parametara:
    positionR = positionL=0;
    L = orientation = 0;
    vR = vL = 0;
}

void LeviPWM(unsigned int PWM)
{
    P1DC1 = PWM;
}

void DesniPWM(unsigned int PWM)
{
    P2DC1 = PWM;
}

static void uart_flush() {
    SRbits.IPL = 7;
    SRbits.IPL = 0;
}
int main(void)
{
	
    /* Configure Oscillator to operate the device at 30Mhz
       Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
       Fosc= 7.37*(32)/(2*2)=58.96Mhz for Fosc, Fcy = 29.48Mhz */

    /* Configure PLL prescaler, PLL postscaler, PLL divisor */
    //PLLFBDbits.PLLDIV=38;   /* M = PLLFBD + 2 */ // izlazna frekvencija = 30Mhz
    //Fin=8MHz, Fcy=30MHz 
	// Configure PLL prescaler, PLL postscaler, PLL divisor
	PLLFBD = 28; 				// M=40    ---> PLLFBD + 2 = M
	CLKDIVbits.PLLPOST = 0; 	// N2=2    ---> 2x(PLLPOST + 2) = N2
	CLKDIVbits.PLLPRE = 0; 	// N1=2    ---> PLLPRE + 2 = N1

	//new oscillator selection
	__builtin_write_OSCCONH(0b011);  				//0b011 ---> XT with PLL
	//enable oscillator source switch
	__builtin_write_OSCCONL (OSCCONL | (1<<0)); 	//OSWEN 

	//wait for PLL lock -> wait to new settings become available
	while (OSCCONbits.COSC != 0b011); 
	//wait for PLL lock
	while (OSCCONbits.LOCK != 0b1); 
    
    AD1PCFGL = 0xFFFF;// all PORT Digital

   
    RPINR18bits.U1RXR = 0;		//UART1 RX na RP0- pin 4
    RPOR0bits.RP1R = 3;			//UART1 TX na RP1- pin 5
    RPINR14bits.QEA1R = 2;		//QEI1A na RP2
    RPINR14bits.QEB1R = 3;		//QEI1B na RP3

    RPINR16bits.QEA2R = 4;		//QEI2A na RP4
    RPINR16bits.QEB2R = 7;		//QEI2B na RP7
    
    //CAN_init(DRIVER_IDENTIFICATOR); // inicijalizacija CAN BUS- a-> argument je adresa drajvera

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
    
    __delay_ms(500);

    while(1) {
        kretanje_pravo(1000, 100);
        __delay_ms(1000);
        
        kretanje_pravo(-1000, 100);
        __delay_ms(1000);
    }
    /*while(1) {
     okret(360);
        __delay_ms(1000);
    }*/
    
    
    while(1)
    {
        // proverava da li ima nes na uartu
        if(DataRdyUART1() > 0) {
            
            // ako ima pogleda sta je prvi byte
            komanda = getch();
            
            switch(komanda)
            {
                // zadavanje pozicije
                case 'I':
                    tmpX = getch_16bit();
                  //  tmpX = -tmpX;
                    tmpY = getch_16bit();
                    //tmpY = -tmpY;
                    tmpO = getch_16bit();
                    
                    
                    setPosition(tmpX, tmpY, tmpO);
                    break;

                // citanje pozicije i statusa
                case 'P':
                    sendStatusAndPosition();
                    break;

                //zadavanje max. brzine (default K2/2)
                case 'V':
                    tmp = getch();
                    setSpeed(tmp);

                    break;

                //kretanje pravo [mm]
                case 'D':
                    tmp = getch_16bit();
                    //tmp = -tmp;
                    v = getch();            // maximal speed
                    PWMinit();              // ako je predhodno bio soft stop da opet inituje
                    kretanje_pravo(tmp, v);

                    break;

                //relativni ugao [stepen]
                case 'T':
                    tmp = getch_16bit();
                    PWMinit();
                    okret(tmp);

                    break;

                //apsolutni ugao [stepen]
                case 'A':
                    tmp = getch_16bit();
                    PWMinit();
                    apsolutni_ugao(tmp);

                    break;

                //idi u tacku (Xc, Yc) [mm]
                case 'G':
                    tmpX = getch_16bit();
                    tmpY = getch_16bit();
                    v = getch();
                    smer = getch();
                    PWMinit();
                    gotoXY(tmpX, tmpY, v, smer);

                    break;

                //kurva
                case 'Q':
                    tmpX = getch_16bit();
                    tmpY = getch_16bit();
                    tmpO = getch_16bit();
                    smer = getch();
                    PWMinit();
                    kurva(tmpX, tmpY, tmpO, smer);

                    break;

                //ukopaj se u mestu (hard stop)
                case 'S':
                   
                    stop();

                    break;

                //stani i ugasi PWM (soft stop)
                case 's':
                    
                    stop();
                    CloseMCPWM();

                    break;

                // kralj meca da se iskljuci pwm
                case 'K':
                    
                    CloseMCPWM();
                    break;
                /*
                default:
                    forceStatus(STATUS_ERROR);
                    break;*/
            }
            
            
        } 
        
        
    }

    return 0;
}
