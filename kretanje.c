#define FCY	30000000ULL

#include <libpic30.h>
#include <xc.h>
#include "globals.h"
#include "kretanje.h"
#include "uart.h"
#include "can.h"
#include "init.h"
#include <math.h>

// set the beinning state
static enum States robot_currentStatus = STATUS_IDLE;

/**
 * @brief reset the driver parameters
 * 
 */
void resetDriver(void)
{
    encoder_rightIncrements = encoder_leftIncrements = 0;
    odometry_incrementsDistance = odometry_incrementsOrientation = 0;
    encoder_rightCurrentIncrements = encoder_leftCurrentIncrements = 0;

    setSpeed(0x80);
    setSpeedAccel(K2);

    setPosition(0, 0, 0);
    robot_currentStatus = STATUS_IDLE;

} // end of resetDriver(...)

/**
 * @brief set the odometry_milliX position of the robot
 * 
 * @param tmp the odometry_milliX value
 */
static void setX(int tmp)
{
    unsigned long t;

    odometry_incrementsX = (long long)tmp * 65534 * K2;
    odometry_refrenceDistance = odometry_incrementsDistance;
    odometry_refrenceOrientation = odometry_incrementsOrientation;

    t = sys_time;
    while(sys_time == t);
} // end of setX(...)

/**
 * @brief set the odometry_milliY position of the robot
 * 
 * @param tmp the odometry_milliY value
 */
static void setY(int tmp)
{
    unsigned long t;

    odometry_incrementsY = (long long)tmp * 65534 * K2;
    odometry_refrenceDistance = odometry_incrementsDistance;
    odometry_refrenceOrientation = odometry_incrementsOrientation;

    t = sys_time;
    while(sys_time == t);
} // end of setY(...)

/**
 * @brief set the odometry_incrementsOrientation of the robot
 * 
 * @param tmp the angle
 */
static void setO(int tmp)
{
    unsigned long t;

    encoder_leftIncrements = -(tmp * K1 / 360) / 2;
    encoder_rightIncrements = (tmp * K1 / 360) / 2;

    odometry_incrementsDistance = (encoder_rightIncrements + encoder_leftIncrements) / 2;
    odometry_incrementsOrientation = (encoder_rightIncrements - encoder_leftIncrements) % K1;

    odometry_refrenceDistance = odometry_incrementsDistance;
    odometry_refrenceOrientation = odometry_incrementsOrientation;

    t = sys_time;
    while(sys_time == t);
} // end of setO(...)

/**
 * @brief Set the Position of the robot
 * 
 * @param odometry_milliX the odometry_milliX position
 * @param odometry_milliY the odometry_milliY position
 * @param odometry_incrementsOrientation the odometry_incrementsOrientation
 */
void setPosition(int odometry_milliX, int odometry_milliY, int odometry_incrementsOrientation)
{
    setX(odometry_milliX);
    setY(odometry_milliY);
    setO(odometry_incrementsOrientation);

    robot_currentStatus = STATUS_IDLE;
} // end of setPosition(...)

/**
 * @brief get the current status/position and send it back
 * 
 */
void sendStatusAndPosition(void)
{
    long robot_currentOrientation = odometry_incrementsOrientation;
    if(robot_currentStatus == STATUS_ERROR)
        putch('E');
    else if(robot_currentStatus == STATUS_MOVING)
        putch('M');
    else if(robot_currentStatus == STATUS_IDLE)
        putch('I');
    else if(robot_currentStatus == STATUS_STUCK)
        putch('S');
    else if(robot_currentStatus == STATUS_ROTATING)
        putch('R');
    
    U1STAbits.OERR = 0;
    
    
    putch_16bit(odometry_milliX);
    putch_16bit(odometry_milliY);

    robot_currentOrientation = ((double)odometry_incrementsOrientation * 360) / K1 + 0.5;
    putch_16bit(robot_currentOrientation);
    
} // end of sendStatusAndPosition(...)

/**
 * @brief set the acceleration parameters
 * 
 * @param v acceleration parameter
 */
void setSpeedAccel(float v)
{
    odometry_speedMax = v;	
    odometry_speedOmega = 2 * odometry_speedMax;
    odometry_acceleration = odometry_speedMax / ROBOT_ACCEL_NUMBER;
    odometry_accelerationAlpha = 2.5 * odometry_acceleration;
} // end of setSpeedAccel(...)

/**
 * @brief helper function for getting the state
 * 
 * @return enum States 
 */
enum States getStatus(void)
{
    return robot_currentStatus;
} // end of getStatus(...)

/**
 * @brief force a status
 * 
 * @param newStatus the new forced status
 */
void forceStatus(enum States newStatus)
{
    robot_currentStatus = newStatus; 
} // end of forceStatus(...)

/**
 * @brief Function for reading command while moving
 * 
 * @return char 
 */
static char getCommand(void)
{
    char uart_command;

   // U1STAbits.OERR = 0;       
    if(DataRdyUART1() > 0)   
    {
        uart_command = getch();

        switch(uart_command)
        {
            case 'P':
                sendStatusAndPosition();
                break;

            case 'S':
                // hard stop
                odometry_refrenceDistance = odometry_incrementsDistance;
                odometry_refrenceOrientation = odometry_incrementsOrientation;
                odometry_refrenceSpeed = 0;

                robot_currentStatus = STATUS_IDLE;
                __delay_ms(10);

                return 0;

            case 's':
                // stop and turn off PWM (stop stop)
                odometry_refrenceDistance = odometry_incrementsDistance;
                odometry_refrenceOrientation = odometry_incrementsOrientation;
                odometry_refrenceSpeed = 0;

                CloseMCPWM();
                robot_currentStatus = STATUS_IDLE;
                __delay_ms(10);

                return 0;

            /*default:
                //case 'G' : case 'D' : case 'T' : case 'A' : case 'Q':
                // primljena komanda za kretanje u toku kretanja

                //dodati da promeni motore...
                
                // stop, status ostaje MOVING

                odometry_refrenceDistance = odometry_incrementsDistance;
                odometry_refrenceOrientation = odometry_incrementsOrientation;
                odometry_refrenceSpeed = 0;

                __delay_ms(100);

                return 0;*/

        } // end of switch(command)
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

    /*if ((odometry_stuckDistance / 128 > brzinaL) || (odometry_stuckOrientation / 128 > brzinaL)) //16,32
    {
        //ukopaj se u mestu
        odometry_refrenceDistance = odometry_incrementsDistance;
        odometry_refrenceOrientation = odometry_incrementsOrientation;
        odometry_refrenceSpeed = 0;

        robot_currentStatus = STATUS_STUCK;
        __delay_ms(50);
        return 0;
    }
*/
    return 1;
} // end of checkStuckCondition(...)

/**
 * @brief goes to XY position at a max speed and in a direction
 * 
 * @param Xd odometry_milliX position
 * @param Yd odometry_milliY position 
 * @param krajnja_brzina end speed 
 * @param smer direction
 */
void gotoXY(int Xd, int Yd, unsigned char krajnja_brzina, char smer)
{
    long t, t0, t1, t2, t3;
    long T1, T2, T3;
    long L_dist, L0, L1, L2, L3;
    long D0, D1, D2;
    float v_encoder_rightCurrentIncrementsh, v_end, v0;
    int duzina, ugao;
    long long int Xdlong, Ydlong;

    Xdlong = (long long)Xd * K2 * 65534;
    Ydlong = (long long)Yd * K2 * 65534;

    v0 = odometry_refrenceSpeed;
    smer = (smer > 0 ? 1 : -1);

    //okreni se prema krajnjoj tacki
    ugao = atan2(Ydlong-odometry_incrementsY, Xdlong-odometry_incrementsX) * (180 / PI) - odometry_incrementsOrientation * 360 / K1;
    if(smer < 0)
        ugao += 180;
    while(ugao > 180)
        ugao -= 360;
    while(ugao < -180)
        ugao += 360;

    if(okret(ugao))
        return;

    duzina = sqrt((odometry_milliX - Xd) * (odometry_milliX - Xd) + (odometry_milliY - Yd) * (odometry_milliY - Yd));

    if(duzina < 500)
        setSpeedAccel(K2/3);

    v_end = odometry_speedMax * krajnja_brzina / 256;
    L_dist = (long)duzina * K2;

    T1 = (odometry_speedMax - odometry_refrenceSpeed) / odometry_acceleration;
    L0 = odometry_incrementsDistance;
    L1 = odometry_refrenceSpeed * T1 + odometry_acceleration * T1 * T1 / 2;

    T3 = (odometry_speedMax - v_end) / odometry_acceleration;
    L3 = odometry_speedMax * T3 - odometry_acceleration * T3 * T3 / 2;

    if( (L1 + L3) < L_dist)
    {
        //moze da dostigne odometry_speedMax
        L2 = L_dist - L1 - L3;
        T2 = L2 / odometry_speedMax;
    }
    else
    {
        //ne moze da dostigne odometry_speedMax
        T2 = 0;
        v_encoder_rightCurrentIncrementsh = sqrt(odometry_acceleration * L_dist + (odometry_refrenceSpeed * odometry_refrenceSpeed + v_end * v_end) / 2);
        if( (v_encoder_rightCurrentIncrementsh < odometry_refrenceSpeed) || (v_encoder_rightCurrentIncrementsh < v_end) )
        {
            robot_currentStatus = STATUS_ERROR;
            return; //mission impossible
        }

        T1 = (v_encoder_rightCurrentIncrementsh - odometry_refrenceSpeed) / odometry_acceleration;
        T3 = (v_encoder_rightCurrentIncrementsh - v_end) / odometry_acceleration;
    }

    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;
    D0 = odometry_refrenceDistance;

    robot_currentStatus = STATUS_MOVING;
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
                    odometry_refrenceOrientation = atan2(Ydlong-odometry_incrementsY, Xdlong-odometry_incrementsX) / (2 * PI) * K1;
                else
                    odometry_refrenceOrientation = atan2(odometry_incrementsY-Ydlong, odometry_incrementsX-Xdlong) / (2 * PI) * K1;
            }

            if(t <= t1)
            {
                odometry_refrenceSpeed = (v0 + odometry_acceleration * (t-t0)) / 1.5;
                D1 = D2 = odometry_refrenceDistance = D0 + smer * (v0 * (t-t0) + odometry_acceleration * (t-t0)*(t-t0)/2);
            }
            else if(t <= t2)
            {
                odometry_refrenceSpeed = odometry_speedMax / 1.2;
                D2 = odometry_refrenceDistance = D1 + smer * odometry_speedMax * (t-t1);
            }
            else if(t <= t3)
            {
                odometry_refrenceSpeed = (odometry_speedMax - odometry_acceleration * (t-t2)) / 2;
                odometry_refrenceDistance = D2 + smer * (odometry_speedMax * (t-t2) - odometry_acceleration * (t-t2) * (t-t2) / 2);
            }
        }

    robot_currentStatus = STATUS_IDLE;
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
    float v_encoder_rightCurrentIncrementsh, v_end, v0;
    char predznak;

   // if((duzina < 500) && (duzina > -500))
    //    setSpeedAccel(K2 / 3);

    v0 = odometry_refrenceSpeed;
    v_end = odometry_speedMax * krajnja_brzina / 255;
    predznak = (duzina >= 0 ? 1 : -1);
    L_dist = (long)duzina * K2; // konverzija u inkremente

    T1 = (odometry_speedMax - odometry_refrenceSpeed) / odometry_acceleration;
    L0 = odometry_incrementsDistance;
    L1 = odometry_refrenceSpeed * T1 + odometry_acceleration * T1 * (T1 / 2);

    T3 = (odometry_speedMax - v_end) / odometry_acceleration;
    L3 = odometry_speedMax * T3 - odometry_acceleration * T3 * (T3 / 2);

    if((L1 + L3) < predznak * L_dist)
    {
        //moze da dostigne odometry_speedMax
        L2 = predznak * L_dist - L1 - L3;
        T2 = L2 / odometry_speedMax;
    }
    else
    {
        //ne moze da dostigne odometry_speedMax
        T2 = 0;
        v_encoder_rightCurrentIncrementsh = sqrt(odometry_acceleration * predznak * L_dist + (odometry_refrenceSpeed * odometry_refrenceSpeed + v_end * v_end) / 2);
        if((v_encoder_rightCurrentIncrementsh < odometry_refrenceSpeed) || (v_encoder_rightCurrentIncrementsh < v_end))
        {
            robot_currentStatus = STATUS_ERROR;
            return; //mission impossible
        }

        T1 = (v_encoder_rightCurrentIncrementsh - odometry_refrenceSpeed) / odometry_acceleration;
        T3 = (v_encoder_rightCurrentIncrementsh - v_end) / odometry_acceleration;
    }

    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;
    D0 = odometry_refrenceDistance;

    robot_currentStatus = STATUS_MOVING;
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
                odometry_refrenceSpeed = v0 + odometry_acceleration * (t-t0);
                D1 = D2 = odometry_refrenceDistance = D0 + predznak * (v0 * (t-t0) + odometry_acceleration * (t-t0)*(t-t0)/2);
            }
            else if(t <= t2)
            {
                odometry_refrenceSpeed = odometry_speedMax;
                D2 = odometry_refrenceDistance = D1 + predznak * odometry_speedMax * (t-t1);
            }
            else if(t <= t3)
            {
                odometry_refrenceSpeed = odometry_speedMax - odometry_acceleration * (t-t2);
                odometry_refrenceDistance = D2 + predznak * (odometry_speedMax * (t-t2) - odometry_acceleration * (t-t2) * (t-t2) / 2);
            }
        }

    robot_currentStatus = STATUS_IDLE;
} // end of kretanje_pravo(...)

/**
 * @brief Absolute angle
 * 
 * @param ugao the angle we want it to send it to
 */
void apsolutni_ugao(int ugao)
{
    int tmp = ugao - odometry_incrementsOrientation * 360 / K1;

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

    T1 = T3 = odometry_speedOmega / odometry_accelerationAlpha;
    Fi1 = odometry_accelerationAlpha * T1 * T1 / 2;
    if(Fi1 > (predznak * Fi_total / 2))
    {
        //trougaoni profil
        Fi1 = predznak  * Fi_total / 2;
        T1 = T3 = sqrt(2 * Fi1 / odometry_accelerationAlpha);
        T2 = 0;
    }
    else
    {
        //trapezni profil
        T2 = (predznak * Fi_total - 2 * Fi1) / odometry_speedOmega;
    }

    ugao_ref = odometry_refrenceOrientation;
    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;

    robot_currentStatus = STATUS_ROTATING;
    while(t < t3)
        if(t != sys_time)
        {
            t = sys_time;
            if(!getCommand())
                return 1;
            //if (t % 16 == 0) putch ((char)(zaglaencoder_rightCurrentIncrements / 16));
            if(!checkStuckCondition())
                return 1;

            if(t <= t1)
            {
                w_ref += odometry_accelerationAlpha;
                ugao_ref += predznak * (w_ref - odometry_accelerationAlpha / 2);
                odometry_refrenceOrientation = ugao_ref;
            }
            else if(t <= t2)
            {
                w_ref = odometry_speedOmega;
                ugao_ref += predznak * odometry_speedOmega;
                odometry_refrenceOrientation = ugao_ref;
            }
            else if(t <= t3)
            {
                w_ref -= odometry_accelerationAlpha;
                ugao_ref += predznak * (w_ref + odometry_accelerationAlpha / 2);
                odometry_refrenceOrientation = ugao_ref;
            }
        }

    robot_currentStatus = STATUS_IDLE;
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
    float v_poc, dist_ref, ugao_ref, w_ref = 0, odometry_refrenceSpeed = 0;
    char predznak;
    int ugao;

    predznak = (Fi >= 0 ? 1 : -1);
    R = sqrt(((odometry_milliX-Xc) * (odometry_milliX-Xc) + (odometry_milliY-Yc) * (odometry_milliY-Yc)));
    Fi_pocetno = atan2(((int)odometry_milliY-(int)Yc), ((int)odometry_milliX-(int)Xc));
    ugao = Fi_pocetno * 180 / PI;
    smer = (smer >= 0 ? 1 : -1);

    ugao = (ugao + smer * predznak * 90) % 360;
    if(ugao > 180)
        ugao -= 360;
    if(ugao < -180)
        ugao += 360;

    ugao -= odometry_incrementsOrientation * 360 / K1;
    ugao %= 360;

    if(ugao > 180)
        ugao -= 360;
    if(ugao < -180)
        ugao += 360;

    okret(ugao);

    v_poc = odometry_speedMax;
    if(odometry_speedMax > K2/32)
        setSpeedAccel(K2 / 32);

    Fi_total = (long)Fi * K1 / 360;

    T1 = T3 = odometry_speedOmega / odometry_accelerationAlpha;
    Fi1 = odometry_accelerationAlpha * T1 * T1 / 2;
    if(Fi1 > (predznak * Fi_total / 2))
    {
        //trougaoni profil
        Fi1 = predznak  * Fi_total / 2;
        T1 = T3 = sqrt(2 * Fi1 / odometry_accelerationAlpha);
        T2 = 0;
    }
    else
    {
        //trapezni profil
        T2 = (predznak * Fi_total - 2 * Fi1) / odometry_speedOmega;
    }

    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;
    ugao_ref = odometry_refrenceOrientation;
    dist_ref = odometry_refrenceDistance;

    robot_currentStatus = STATUS_MOVING;
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
                w_ref += odometry_accelerationAlpha;
                odometry_refrenceSpeed += odometry_acceleration;
                delta = predznak * (w_ref - odometry_accelerationAlpha / 2);
                luk = predznak * R * delta / D_tocka;
                ugao_ref += delta;
                dist_ref += smer * luk;
                odometry_refrenceOrientation = ugao_ref;
                odometry_refrenceDistance = dist_ref;
            }
            else if(t <= t2)
            {
                w_ref = odometry_speedOmega;
                odometry_refrenceSpeed = odometry_speedMax;
                delta = predznak * odometry_speedOmega;
                luk = predznak * R * delta / D_tocka;
                ugao_ref += delta;
                dist_ref += smer * luk;
                odometry_refrenceOrientation = ugao_ref;
                odometry_refrenceDistance = dist_ref;
            }
            else if(t <= t3)
            {
                w_ref -= odometry_accelerationAlpha;
                odometry_refrenceSpeed -= odometry_acceleration;
                delta = predznak * (w_ref + odometry_accelerationAlpha / 2);
                luk = predznak * R * delta / D_tocka;
                ugao_ref += delta;
                dist_ref += smer * luk;
                odometry_refrenceOrientation = ugao_ref;
                odometry_refrenceDistance = dist_ref;
            }
        }

    setSpeedAccel(v_poc);
    robot_currentStatus = STATUS_IDLE;
} // end of kurva(...)
 
/**
 * @brief stop the robot at the current point
 * 
 */
void stop(void)
{
    odometry_refrenceDistance = odometry_incrementsDistance;
    odometry_refrenceOrientation = odometry_incrementsOrientation;

    robot_currentStatus = STATUS_IDLE;
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
