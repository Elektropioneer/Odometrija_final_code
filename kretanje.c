#define FCY	30000000ULL

#include <libpic30.h>
#include <xc.h>
#include "globals.h"
#include "kretanje.h"
#include "uart.h"
#include "can.h"
#include <math.h>

// set the beinning state
static enum States currentStatus = STATUS_IDLE;

/**
 * @brief reset the driver parameters
 * 
 */
void resetDriver(void)
{
    positionR = positionL = 0;
    L = orientation = 0;
    vR = vL = 0;

    setSpeed(0x80);
    setSpeedAccel(K2);

    setPosition(0, 0, 0);
    currentStatus = STATUS_IDLE;

} // end of resetDriver(...)

/**
 * @brief set the X position of the robot
 * 
 * @param tmp the X value
 */
static void setX(int tmp)
{
    unsigned long t;

    Xlong = (long long)tmp * 65534 * K2;
    d_ref = L;
    t_ref = orientation;

    t = sys_time;
    while(sys_time == t);
} // end of setX(...)

/**
 * @brief set the Y position of the robot
 * 
 * @param tmp the Y value
 */
static void setY(int tmp)
{
    unsigned long t;

    Ylong = (long long)tmp * 65534 * K2;
    d_ref = L;
    t_ref = orientation;

    t = sys_time;
    while(sys_time == t);
} // end of setY(...)

/**
 * @brief set the orientation of the robot
 * 
 * @param tmp the angle
 */
static void setO(int tmp)
{
    unsigned long t;

    positionL = -(tmp * K1 / 360) / 2;
    positionR = (tmp * K1 / 360) / 2;

    L = (positionR + positionL) / 2;
    orientation = (positionR - positionL) % K1;

    d_ref = L;
    t_ref = orientation;

    t = sys_time;
    while(sys_time == t);
} // end of setO(...)

/**
 * @brief Set the Position of the robot
 * 
 * @param X the X position
 * @param Y the Y position
 * @param orientation the orientation
 */
void setPosition(int X, int Y, int orientation)
{
    setX(X);
    setY(Y);
    setO(orientation);

    currentStatus = STATUS_IDLE;
} // end of setPosition(...)

/**
 * @brief get the current status/position and send it back
 * 
 */
void sendStatusAndPosition(void)
{
    long tmpO = orientation;
    if(currentStatus == STATUS_ERROR)
        putch('E');
    else if(currentStatus == STATUS_MOVING)
        putch('M');
    else if(currentStatus == STATUS_IDLE)
        putch('I');
    else if(currentStatus == STATUS_STUCK)
        putch('S');
    else if(currentStatus == STATUS_ROTATING)
        putch('R');
    
    U1STAbits.OERR = 0;
    
    
    putch_16bit(X);
    putch_16bit(Y);

    tmpO = ((double)orientation * 360) / K1 + 0.5;
    putch_16bit(tmpO);
    
} // end of sendStatusAndPosition(...)

/**
 * @brief set the acceleration parameters
 * 
 * @param v acceleration parameter
 */
void setSpeedAccel(float v)
{
    vmax = v;	
    omega = 2 * vmax;
    accel = vmax / ROBOT_ACCEL_NUMBER;
    alfa = 2.5 * accel;
} // end of setSpeedAccel(...)

/**
 * @brief helper function for getting the state
 * 
 * @return enum States 
 */
enum States getStatus(void)
{
    return currentStatus;
} // end of getStatus(...)

/**
 * @brief force a status
 * 
 * @param newStatus the new forced status
 */
void forceStatus(enum States newStatus)
{
    currentStatus = newStatus; 
} // end of forceStatus(...)

/**
 * @brief Function for reading command while moving
 * 
 * @return char 
 */
static char getCommand(void)
{
    char command;

   // U1STAbits.OERR = 0;       
    if(DataRdyUART1() > 0)   
    {
        command = getch();

        switch(command)
        {
            case 'P':
                sendStatusAndPosition();
                break;

            case 'S':
                // hard stop
                d_ref = L;
                t_ref = orientation;
                v_ref = 0;

                currentStatus = STATUS_IDLE;
                __delay_ms(10);

                return 0;

            case 's':
                // stop and turn off PWM (stop stop)
                d_ref = L;
                t_ref = orientation;
                v_ref = 0;

                CloseMCPWM();
                currentStatus = STATUS_IDLE;
                __delay_ms(10);

                return 0;

            /*default:
                //case 'G' : case 'D' : case 'T' : case 'A' : case 'Q':
                // primljena komanda za kretanje u toku kretanja

                //dodati da promeni motore...
                
                // stop, status ostaje MOVING

                d_ref = L;
                t_ref = orientation;
                v_ref = 0;

                __delay_ms(100);

                return 0;*/

        }// end of switch(command)
    }

    return 1;

} // end of getCommand(...)

/**
 * @brief checks if we are stuck (needs testing)
 * 
 * @return char 
 */
static char checkStuckCondition(void)
{

    /*if ((zaglavL / 128 > brzinaL) || (zaglavR / 128 > brzinaL)) //16,32
    {
        //ukopaj se u mestu
        d_ref = L;
        t_ref = orientation;
        v_ref = 0;

        currentStatus = STATUS_STUCK;
        __delay_ms(50);
        return 0;
    }
*/
    return 1;
} // end of checkStuckCondition(...)

/**
 * @brief goes to XY position at a max speed and in a direction
 * 
 * @param Xd X position
 * @param Yd Y position 
 * @param krajnja_brzina end speed 
 * @param smer direction
 */
void gotoXY(int Xd, int Yd, unsigned char krajnja_brzina, char smer)
{
    long t, t0, t1, t2, t3;
    long T1, T2, T3;
    long L_dist, L0, L1, L2, L3;
    long D0, D1, D2;
    float v_vrh, v_end, v0;
    int duzina, ugao;
    long long int Xdlong, Ydlong;

    Xdlong = (long long)Xd * K2 * 65534;
    Ydlong = (long long)Yd * K2 * 65534;

    v0 = v_ref;
    smer = (smer > 0 ? 1 : -1);

    //okreni se prema krajnjoj tacki
    ugao = atan2(Ydlong-Ylong, Xdlong-Xlong) * (180 / PI) - orientation * 360 / K1;
    if(smer < 0)
        ugao += 180;
    while(ugao > 180)
        ugao -= 360;
    while(ugao < -180)
        ugao += 360;

    if(okret(ugao))
        return;

    duzina = sqrt((X - Xd) * (X - Xd) + (Y - Yd) * (Y - Yd));

    if(duzina < 500)
        setSpeedAccel(K2/3);

    v_end = vmax * krajnja_brzina / 256;
    L_dist = (long)duzina * K2;

    T1 = (vmax - v_ref) / accel;
    L0 = L;
    L1 = v_ref * T1 + accel * T1 * T1 / 2;

    T3 = (vmax - v_end) / accel;
    L3 = vmax * T3 - accel * T3 * T3 / 2;

    if( (L1 + L3) < L_dist)
    {
        //moze da dostigne vmax
        L2 = L_dist - L1 - L3;
        T2 = L2 / vmax;
    }
    else
    {
        //ne moze da dostigne vmax
        T2 = 0;
        v_vrh = sqrt(accel * L_dist + (v_ref * v_ref + v_end * v_end) / 2);
        if( (v_vrh < v_ref) || (v_vrh < v_end) )
        {
            currentStatus = STATUS_ERROR;
            return; //mission impossible
        }

        T1 = (v_vrh - v_ref) / accel;
        T3 = (v_vrh - v_end) / accel;
    }

    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;
    D0 = d_ref;

    currentStatus = STATUS_MOVING;
    while(t < t3)
        if(t != sys_time)
        {
            t = sys_time;
            if(!getCommand()) {
                return;
            }

            if (!checkStuckCondition())
                return;

            if(t <= t2)
            {
                if(smer > 0)
                    t_ref = atan2(Ydlong-Ylong, Xdlong-Xlong) / (2 * PI) * K1;
                else
                    t_ref = atan2(Ylong-Ydlong, Xlong-Xdlong) / (2 * PI) * K1;
            }

            if(t <= t1)
            {
                v_ref = (v0 + accel * (t-t0)) / 1.5;
                D1 = D2 = d_ref = D0 + smer * (v0 * (t-t0) + accel * (t-t0)*(t-t0)/2);
            }
            else if(t <= t2)
            {
                v_ref = vmax / 1.2;
                D2 = d_ref = D1 + smer * vmax * (t-t1);
            }
            else if(t <= t3)
            {
                v_ref = (vmax - accel * (t-t2)) / 2;
                d_ref = D2 + smer * (vmax * (t-t2) - accel * (t-t2) * (t-t2) / 2);
            }
        }

    currentStatus = STATUS_IDLE;
} // end of gotoXY(...)


/**
 * @brief move the robot forward/backward
 * 
 * @param duzina the length in [mm]
 * @param krajnja_brzina end speed
 */
void kretanje_pravo(int duzina, unsigned char krajnja_brzina)
{
    long t, t0, t1, t2, t3;
    long T1, T2, T3;
    long L_dist, L0, L1, L2, L3;
    long D0, D1, D2;
    float v_vrh, v_end, v0;
    char predznak;

   // if((duzina < 500) && (duzina > -500))
    //    setSpeedAccel(K2 / 3);

    v0 = v_ref;
    v_end = vmax * krajnja_brzina / 255;
    predznak = (duzina >= 0 ? 1 : -1);
    L_dist = (long)duzina * K2; // konverzija u inkremente

    T1 = (vmax - v_ref) / accel;
    L0 = L;
    L1 = v_ref * T1 + accel * T1 * (T1 / 2);

    T3 = (vmax - v_end) / accel;
    L3 = vmax * T3 - accel * T3 * (T3 / 2);

    if((L1 + L3) < predznak * L_dist)
    {
        //moze da dostigne vmax
        L2 = predznak * L_dist - L1 - L3;
        T2 = L2 / vmax;
    }
    else
    {
        //ne moze da dostigne vmax
        T2 = 0;
        v_vrh = sqrt(accel * predznak * L_dist + (v_ref * v_ref + v_end * v_end) / 2);
        if((v_vrh < v_ref) || (v_vrh < v_end))
        {
            currentStatus = STATUS_ERROR;
            return; //mission impossible
        }

        T1 = (v_vrh - v_ref) / accel;
        T3 = (v_vrh - v_end) / accel;
    }

    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;
    D0 = d_ref;

    currentStatus = STATUS_MOVING;
    while(t < t3)
        if(t != sys_time)
        {
            t = sys_time;
            if(!getCommand())
                return;

            if (!checkStuckCondition())
                return;

            if(t <= t1)
            {
                v_ref = v0 + accel * (t-t0);
                D1 = D2 = d_ref = D0 + predznak * (v0 * (t-t0) + accel * (t-t0)*(t-t0)/2);
            }
            else if(t <= t2)
            {
                v_ref = vmax;
                D2 = d_ref = D1 + predznak * vmax * (t-t1);
            }
            else if(t <= t3)
            {
                v_ref = vmax - accel * (t-t2);
                d_ref = D2 + predznak * (vmax * (t-t2) - accel * (t-t2) * (t-t2) / 2);
            }
        }

    currentStatus = STATUS_IDLE;
} // end of kretanje_pravo(...)

/**
 * @brief Absolute angle
 * 
 * @param ugao the angle we want it to send it to
 */
void apsolutni_ugao(int ugao)
{
    int tmp = ugao - orientation * 360 / K1;

    if(tmp > 180)
        tmp -= 360;
    if(tmp <-180)
        tmp += 360;

    okret(tmp);
} // end of apsolutni_ugao(...)

/**
 * @brief turn the robot at a specific angle (relative)
 * 
 * @param ugao the angle
 * @return char 
 */
char okret(int ugao)
{
    long t, t0, t1, t2, t3;
    long T1, T2, T3;
    long Fi_total, Fi1;
    float ugao_ref, w_ref = 0;
    char predznak;

    predznak = (ugao >= 0 ? 1 : -1);
    Fi_total = (long)ugao * K1 / 360;

    T1 = T3 = omega / alfa;
    Fi1 = alfa * T1 * T1 / 2;
    if(Fi1 > (predznak * Fi_total / 2))
    {
        //trougaoni profil
        Fi1 = predznak  * Fi_total / 2;
        T1 = T3 = sqrt(2 * Fi1 / alfa);
        T2 = 0;
    }
    else
    {
        //trapezni profil
        T2 = (predznak * Fi_total - 2 * Fi1) / omega;
    }

    ugao_ref = t_ref;
    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;

    currentStatus = STATUS_ROTATING;
    while(t < t3)
        if(t != sys_time)
        {
            t = sys_time;
            if(!getCommand())
                return 1;
            //if (t % 16 == 0) putch ((char)(zaglavR / 16));
            if(!checkStuckCondition())
                return 1;

            if(t <= t1)
            {
                w_ref += alfa;
                ugao_ref += predznak * (w_ref - alfa / 2);
                t_ref = ugao_ref;
            }
            else if(t <= t2)
            {
                w_ref = omega;
                ugao_ref += predznak * omega;
                t_ref = ugao_ref;
            }
            else if(t <= t3)
            {
                w_ref -= alfa;
                ugao_ref += predznak * (w_ref + alfa / 2);
                t_ref = ugao_ref;
            }
        }

    currentStatus = STATUS_IDLE;
    return 0;
} // end of okret(...)

/**
 * @brief do a kurva motion
 * 
 * @param Xc 
 * @param Yc 
 * @param Fi 
 * @param smer 
 */
void kurva(long Xc, long Yc, int Fi, char smer)
{
    float R, Fi_pocetno, delta, luk;
    long t, t0, t1, t2, t3;
    long T1, T2, T3;
    long Fi_total, Fi1;
    float v_poc, dist_ref, ugao_ref, w_ref = 0, v_ref = 0;
    char predznak;
    int ugao;

    predznak = (Fi >= 0 ? 1 : -1);
    R = sqrt(((X-Xc) * (X-Xc) + (Y-Yc) * (Y-Yc)));
    Fi_pocetno = atan2(((int)Y-(int)Yc), ((int)X-(int)Xc));
    ugao = Fi_pocetno * 180 / PI;
    smer = (smer >= 0 ? 1 : -1);

    ugao = (ugao + smer * predznak * 90) % 360;
    if(ugao > 180)
        ugao -= 360;
    if(ugao < -180)
        ugao += 360;

    ugao -= orientation * 360 / K1;
    ugao %= 360;

    if(ugao > 180)
        ugao -= 360;
    if(ugao < -180)
        ugao += 360;

    okret(ugao);

    v_poc = vmax;
    if(vmax > K2/32)
        setSpeedAccel(K2 / 32);

    Fi_total = (long)Fi * K1 / 360;

    T1 = T3 = omega / alfa;
    Fi1 = alfa * T1 * T1 / 2;
    if(Fi1 > (predznak * Fi_total / 2))
    {
        //trougaoni profil
        Fi1 = predznak  * Fi_total / 2;
        T1 = T3 = sqrt(2 * Fi1 / alfa);
        T2 = 0;
    }
    else
    {
        //trapezni profil
        T2 = (predznak * Fi_total - 2 * Fi1) / omega;
    }

    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;
    ugao_ref = t_ref;
    dist_ref = d_ref;

    currentStatus = STATUS_MOVING;
    while(t < t3)
        if(t != sys_time)
        {
            t = sys_time;
            if(!getCommand())
            {
                setSpeedAccel(v_poc);
                return;
            }

            if(!checkStuckCondition())
            {
                setSpeedAccel(v_poc);
                return;
            }

            if(t <= t1)
            {
                w_ref += alfa;
                v_ref += accel;
                delta = predznak * (w_ref - alfa / 2);
                luk = predznak * R * delta / D_tocka;
                ugao_ref += delta;
                dist_ref += smer * luk;
                t_ref = ugao_ref;
                d_ref = dist_ref;
            }
            else if(t <= t2)
            {
                w_ref = omega;
                v_ref = vmax;
                delta = predznak * omega;
                luk = predznak * R * delta / D_tocka;
                ugao_ref += delta;
                dist_ref += smer * luk;
                t_ref = ugao_ref;
                d_ref = dist_ref;
            }
            else if(t <= t3)
            {
                w_ref -= alfa;
                v_ref -= accel;
                delta = predznak * (w_ref + alfa / 2);
                luk = predznak * R * delta / D_tocka;
                ugao_ref += delta;
                dist_ref += smer * luk;
                t_ref = ugao_ref;
                d_ref = dist_ref;
            }
        }

    setSpeedAccel(v_poc);
    currentStatus = STATUS_IDLE;
} // end of kurva(...)
 
/**
 * @brief stop the robot at the current point
 * 
 */
void stop(void)
{
    d_ref = L;
    t_ref = orientation;

    currentStatus = STATUS_IDLE;
} // end of stop(...)

/**
 * @brief Set the max speed of the robot
 * 
 * @param tmp the max speed
 */
void setSpeed(unsigned char tmp)
{
    brzinaL = tmp;
    setSpeedAccel(K2  * (unsigned char)brzinaL / 256);
} // end of setSpeed(...)
