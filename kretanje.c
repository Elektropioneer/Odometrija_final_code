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
    unsigned long current_time;

    odometry_incrementsX = (long long)tmp * 65534 * K2;
    odometry_refrenceDistance = odometry_incrementsDistance;
    odometry_refrenceOrientation = odometry_incrementsOrientation;

    current_time = sys_time;
    while(sys_time == current_time);
} // end of setX(...)

/**
 * @brief set the odometry_milliY position of the robot
 * 
 * @param tmp the odometry_milliY value
 */
static void setY(int tmp)
{
    unsigned long current_time;

    odometry_incrementsY = (long long)tmp * 65534 * K2;
    odometry_refrenceDistance = odometry_incrementsDistance;
    odometry_refrenceOrientation = odometry_incrementsOrientation;

    current_time = sys_time;
    while(sys_time == current_time);
} // end of setY(...)

/**
 * @brief set the odometry_incrementsOrientation of the robot
 * 
 * @param tmp the angle
 */
static void setO(int tmp)
{
    unsigned long current_time;

    encoder_leftIncrements = -(tmp * K1 / 360) / 2;
    encoder_rightIncrements = (tmp * K1 / 360) / 2;

    odometry_incrementsDistance = (encoder_rightIncrements + encoder_leftIncrements) / 2;
    odometry_incrementsOrientation = (encoder_rightIncrements - encoder_leftIncrements) % K1;

    odometry_refrenceDistance = odometry_incrementsDistance;
    odometry_refrenceOrientation = odometry_incrementsOrientation;

    current_time = sys_time;
    while(sys_time == current_time);
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

    /*if ((odometry_stuckDistance / 128 > odometry_maxSpeedSet) || (odometry_stuckOrientation / 128 > odometry_maxSpeedSet)) //16,32
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
 * @param robot_maxSpeed end speed 
 * @param robot_movingDirection direction
 */
void gotoXY(int Xd, int Yd, unsigned char robot_maxSpeed, char robot_movingDirection)
{
    long current_time, current_time0, time_stage1, time_stage2, time_stage3;
    long time_calculatedStage1, time_calculatedStage2, time_calculatedStage3;
    long distance_Total, distance_currentIncrements, distance_stage1Increments, distance_stage2Increments, distance_stage3Increments;
    long distance_stage1, distance_stage2, distance_stage3;
    float v_vrh, v_end, current_speed;
    int robot_distance,  robot_orientation,   t_orientation;
    long long int position_futureIncrementsX, position_futureIncrementsY;

    // converting our future position by Xd and Yd to increments
    position_futureIncrementsX = (long long)Xd * K2 * 65534;
    position_futureIncrementsY = (long long)Yd * K2 * 65534;

    // get the current speed
    current_speed = odometry_refrenceSpeed;

    // calculate the direction (positive/negative)
    robot_movingDirection = (robot_movingDirection > 0 ? 1 : -1);

    // calculate the orientation angle to turn to 
    robot_orientation = atan2(position_futureIncrementsY-odometry_incrementsY, position_futureIncrementsX-odometry_incrementsX) * (180 / PI) - odometry_incrementsOrientation * 360 / K1;
    
    if(robot_movingDirection < 0)
        robot_orientation += 180;
    while(robot_orientation > 180)
        robot_orientation -= 360;
    while(robot_orientation < -180)
        robot_orientation += 360;

    // do the orientation move
    if(okret(robot_orientation))
        return;

    // calculate the distance we need to go forward to achiveve our position
    robot_distance = sqrt((odometry_milliX - Xd) * (odometry_milliX - Xd) + (odometry_milliY - Yd) * (odometry_milliY - Yd));

    // if the distance is less than 500mm, don'current_time accelerate so hard
    if(robot_distance < 500)
        setSpeedAccel(K2/3);

    // calculate the end speed we should have
    v_end = odometry_speedMax * robot_maxSpeed / 256;
    distance_Total = (long)robot_distance * K2;                     // calculate the distance we will conver [increments]

    time_calculatedStage1 = (odometry_speedMax - odometry_refrenceSpeed) / odometry_acceleration;  // calculate the time it will take to speed up
    distance_currentIncrements = odometry_incrementsDistance;                                           // the whole distance to cover
    distance_stage1Increments = odometry_refrenceSpeed * time_calculatedStage1 + odometry_acceleration * time_calculatedStage1 * time_calculatedStage1 / 2;     // calculate the distance to speed up

    time_calculatedStage3 = (odometry_speedMax - v_end) / odometry_acceleration;                   // the time it will take to deaccelerate
    distance_stage3Increments = odometry_speedMax * time_calculatedStage3 - odometry_acceleration * time_calculatedStage3 * time_calculatedStage3 / 2;          // the distance it will take to stop

    // can we achieve max speed
    if( (distance_stage1Increments + distance_stage3Increments) < distance_Total)
    {   
        // we can achieve max speed
        distance_stage2Increments = distance_Total - distance_stage1Increments - distance_stage3Increments;                                          // the distance we will hold max speed
        time_calculatedStage2 = distance_stage2Increments / odometry_speedMax;                 // the time we will hold max speed
    }
    else
    {
        // we can't achieve max speed
        time_calculatedStage2 = 0;                                     // the time we will hold max speed = 0
        
        // calculate the new possible max speed we can achieve
        v_vrh = sqrt(odometry_acceleration * distance_Total + (odometry_refrenceSpeed * odometry_refrenceSpeed + v_end * v_end) / 2);
        
        if( (v_vrh < odometry_refrenceSpeed) || (v_vrh < v_end) )
        {
            robot_currentStatus = STATUS_ERROR;
            return; //mission impossible
        }

        // change the time of speed up/down depending on the new max speed
        time_calculatedStage1 = (v_vrh - odometry_refrenceSpeed) / odometry_acceleration;
        time_calculatedStage3 = (v_vrh - v_end) / odometry_acceleration;
    }

    current_time = current_time0 = sys_time;                      // get the current time
    time_stage1 = current_time0 + time_calculatedStage1;                           // time it will take to speed up in ms
    time_stage2 = time_stage1 + time_calculatedStage2;                           // time we can hold the max speed in ms
    time_stage3 = time_stage2 + time_calculatedStage3;                           // time it will take to slow down in ms
    distance_stage1 = odometry_refrenceDistance;         // current position

    // update robot status to moving
    robot_currentStatus = STATUS_MOVING;

    // while we haven'current_time gotten to stage 3 
    while(current_time < time_stage3) {
        // update once per ms
        if(current_time != sys_time)
        {
            // get the sys_time
            current_time = sys_time;   

            // check if communication has arrived
            if(!getCommand()) {
                return;
            }

            // check if the robot is stuck
            if (!checkStuckCondition())
                return;

            // orientation  update (on every stage)
            if(current_time <= time_stage2)
            {
                // depending on moving direction, the angle will change
                if(robot_movingDirection > 0)
                    odometry_refrenceOrientation = atan2(position_futureIncrementsY-odometry_incrementsY, position_futureIncrementsX-odometry_incrementsX) / (2 * PI) * K1;
                else
                    odometry_refrenceOrientation = atan2(odometry_incrementsY-position_futureIncrementsY, odometry_incrementsX-position_futureIncrementsX) / (2 * PI) * K1;
            }

            // speed up stage
            if(current_time <= time_stage1)
            {
                // update our speed
                odometry_refrenceSpeed = (current_speed + odometry_acceleration * (current_time-current_time0)) / 1.5;
                
                // update the refrence distance
                distance_stage2 = distance_stage3 = odometry_refrenceDistance = distance_stage1 + robot_movingDirection * (current_speed * (current_time-current_time0) + odometry_acceleration * (current_time-current_time0)*(current_time-current_time0)/2);
            }
            // hold max speed stage
            else if(current_time <= time_stage2)
            {
                // update our speed
                odometry_refrenceSpeed = odometry_speedMax / 1.2;

                // update the refrence distance
                distance_stage3 = odometry_refrenceDistance = distance_stage2 + robot_movingDirection * odometry_speedMax * (current_time-time_stage1);
            }
            // slow down stage
            else if(current_time <= time_stage3)
            {
                // update our speed
                odometry_refrenceSpeed = (odometry_speedMax - odometry_acceleration * (current_time-time_stage2)) / 2;
                
                // update the refrence distance
                odometry_refrenceDistance = distance_stage3 + robot_movingDirection * (odometry_speedMax * (current_time-time_stage2) - odometry_acceleration * (current_time-time_stage2) * (current_time-time_stage2) / 2);
            }
        }
    }
    // arrived to destination, idle it
    robot_currentStatus = STATUS_IDLE;
} // end of gotoXY(...)


/**
 * @brief move the robot forward/backward
 * 
 * @param robot_distance the length in [mm]
 * @param robot_maxSpeed end speed
 */
void kretanje_pravo(int robot_distance, unsigned char robot_maxSpeed)
{
    long current_time, current_time0, time_stage1, time_stage2, time_stage3;
    long time_calculatedStage1, time_calculatedStage2, time_calculatedStage3;
    long distance_Total, distance_currentIncrements, distance_stage1Increments, distance_stage2Increments, distance_stage3Increments;
    long distance_stage1, distance_stage2, distance_stage3;
    float v_vrh, v_end, current_speed;
    char predznak;

   // if((robot_distance < 500) && (robot_distance > -500))
    //    setSpeedAccel(K2 / 3);

    current_speed = odometry_refrenceSpeed;
    v_end = odometry_speedMax * robot_maxSpeed / 255;
    predznak = (robot_distance >= 0 ? 1 : -1);
    distance_Total = (long)robot_distance * K2; // konverzija u inkremente

    time_calculatedStage1 = (odometry_speedMax - odometry_refrenceSpeed) / odometry_acceleration;
    distance_currentIncrements = odometry_incrementsDistance;
    distance_stage1Increments = odometry_refrenceSpeed * time_calculatedStage1 + odometry_acceleration * time_calculatedStage1 * (time_calculatedStage1 / 2);

    time_calculatedStage3 = (odometry_speedMax - v_end) / odometry_acceleration;
    distance_stage3Increments = odometry_speedMax * time_calculatedStage3 - odometry_acceleration * time_calculatedStage3 * (time_calculatedStage3 / 2);

    if((distance_stage1Increments + distance_stage3Increments) < predznak * distance_Total)
    {
        //moze da dostigne odometry_speedMax
        distance_stage2Increments = predznak * distance_Total - distance_stage1Increments - distance_stage3Increments;
        time_calculatedStage2 = distance_stage2Increments / odometry_speedMax;
    }
    else
    {
        //ne moze da dostigne odometry_speedMax
        time_calculatedStage2 = 0;
        v_vrh = sqrt(odometry_acceleration * predznak * distance_Total + (odometry_refrenceSpeed * odometry_refrenceSpeed + v_end * v_end) / 2);
        if((v_vrh < odometry_refrenceSpeed) || (v_vrh < v_end))
        {
            robot_currentStatus = STATUS_ERROR;
            return; //mission impossible
        }

        time_calculatedStage1 = (v_vrh - odometry_refrenceSpeed) / odometry_acceleration;
        time_calculatedStage3 = (v_vrh - v_end) / odometry_acceleration;
    }

    current_time = current_time0 = sys_time;
    time_stage1 = current_time0 + time_calculatedStage1;
    time_stage2 = time_stage1 + time_calculatedStage2;
    time_stage3 = time_stage2 + time_calculatedStage3;
    distance_stage1 = odometry_refrenceDistance;

    robot_currentStatus = STATUS_MOVING;
    while(current_time < time_stage3) {
        if(current_time != sys_time)
        {
            current_time = sys_time;
            if(!getCommand())
                return;

            if (!checkStuckCondition())
                return;

            if(current_time <= time_stage1)
            {
                odometry_refrenceSpeed = current_speed + odometry_acceleration * (current_time-current_time0);
                distance_stage2 = distance_stage3 = odometry_refrenceDistance = distance_stage1 + predznak * (current_speed * (current_time-current_time0) + odometry_acceleration * (current_time-current_time0)*(current_time-current_time0)/2);
            }
            else if(current_time <= time_stage2)
            {
                odometry_refrenceSpeed = odometry_speedMax;
                distance_stage3 = odometry_refrenceDistance = distance_stage2 + predznak * odometry_speedMax * (current_time-time_stage1);
            }
            else if(current_time <= time_stage3)
            {
                odometry_refrenceSpeed = odometry_speedMax - odometry_acceleration * (current_time-time_stage2);
                odometry_refrenceDistance = distance_stage3 + predznak * (odometry_speedMax * (current_time-time_stage2) - odometry_acceleration * (current_time-time_stage2) * (current_time-time_stage2) / 2);
            }
        }
    }
    robot_currentStatus = STATUS_IDLE;
} // end of kretanje_pravo(...)

/**
 * @brief Absolute angle
 * 
 * @param robot_orientation the angle we want it to send it to
 */
void apsolutni_ugao(int robot_orientation)
{
    int tmp = robot_orientation - odometry_incrementsOrientation * 360 / K1;

    if(tmp > 180)
        tmp -= 360;
    if(tmp <-180)
        tmp += 360;

    okret(tmp);
} // end of apsolutni_ugao(...)

/**
 * @brief turn the robot at a specific angle (relative)
 * 
 * @param robot_orientation the angle
 * @return char 
 */
char okret(int robot_orientation)
{
    long current_time, current_time0, time_stage1, time_stage2, time_stage3;
    long time_calculatedStage1, time_calculatedStage2, time_calculatedStage3;
    long Fi_total, Fi1;
    float ugao_ref, w_ref = 0;
    char predznak;

    predznak = (robot_orientation >= 0 ? 1 : -1);
    Fi_total = (long)robot_orientation * K1 / 360;

    time_calculatedStage1 = time_calculatedStage3 = odometry_speedOmega / odometry_accelerationAlpha;
    Fi1 = odometry_accelerationAlpha * time_calculatedStage1 * time_calculatedStage1 / 2;
    if(Fi1 > (predznak * Fi_total / 2))
    {
        //trougaoni profil
        Fi1 = predznak  * Fi_total / 2;
        time_calculatedStage1 = time_calculatedStage3 = sqrt(2 * Fi1 / odometry_accelerationAlpha);
        time_calculatedStage2 = 0;
    }
    else
    {
        //trapezni profil
        time_calculatedStage2 = (predznak * Fi_total - 2 * Fi1) / odometry_speedOmega;
    }

    ugao_ref = odometry_refrenceOrientation;
    current_time = current_time0 = sys_time;
    time_stage1 = current_time0 + time_calculatedStage1;
    time_stage2 = time_stage1 + time_calculatedStage2;
    time_stage3 = time_stage2 + time_calculatedStage3;

    robot_currentStatus = STATUS_ROTATING;
    while(current_time < time_stage3)
        if(current_time != sys_time)
        {
            current_time = sys_time;
            if(!getCommand())
                return 1;
            //if (current_time % 16 == 0) putch ((char)(zaglaencoder_rightCurrentIncrements / 16));
            if(!checkStuckCondition())
                return 1;

            if(current_time <= time_stage1)
            {
                w_ref += odometry_accelerationAlpha;
                ugao_ref += predznak * (w_ref - odometry_accelerationAlpha / 2);
                odometry_refrenceOrientation = ugao_ref;
            }
            else if(current_time <= time_stage2)
            {
                w_ref = odometry_speedOmega;
                ugao_ref += predznak * odometry_speedOmega;
                odometry_refrenceOrientation = ugao_ref;
            }
            else if(current_time <= time_stage3)
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
 * @param robot_movingDirection 
 */
void kurva(long Xc, long Yc, int Fi, char robot_movingDirection)
{
    float R, Fi_pocetno, delta, luk;
    long current_time, current_time0, time_stage1, time_stage2, time_stage3;
    long time_calculatedStage1, time_calculatedStage2, time_calculatedStage3;
    long Fi_total, Fi1;
    float v_poc, dist_ref, ugao_ref, w_ref = 0, odometry_refrenceSpeed = 0;
    char predznak;
    int robot_orientation;

    predznak = (Fi >= 0 ? 1 : -1);
    R = sqrt(((odometry_milliX-Xc) * (odometry_milliX-Xc) + (odometry_milliY-Yc) * (odometry_milliY-Yc)));
    Fi_pocetno = atan2(((int)odometry_milliY-(int)Yc), ((int)odometry_milliX-(int)Xc));
    robot_orientation = Fi_pocetno * 180 / PI;
    robot_movingDirection = (robot_movingDirection >= 0 ? 1 : -1);

    robot_orientation = (robot_orientation + robot_movingDirection * predznak * 90) % 360;
    if(robot_orientation > 180)
        robot_orientation -= 360;
    if(robot_orientation < -180)
        robot_orientation += 360;

    robot_orientation -= odometry_incrementsOrientation * 360 / K1;
    robot_orientation %= 360;

    if(robot_orientation > 180)
        robot_orientation -= 360;
    if(robot_orientation < -180)
        robot_orientation += 360;

    okret(robot_orientation);

    v_poc = odometry_speedMax;
    if(odometry_speedMax > K2/32)
        setSpeedAccel(K2 / 32);

    Fi_total = (long)Fi * K1 / 360;

    time_calculatedStage1 = time_calculatedStage3 = odometry_speedOmega / odometry_accelerationAlpha;
    Fi1 = odometry_accelerationAlpha * time_calculatedStage1 * time_calculatedStage1 / 2;
    if(Fi1 > (predznak * Fi_total / 2))
    {
        //trougaoni profil
        Fi1 = predznak  * Fi_total / 2;
        time_calculatedStage1 = time_calculatedStage3 = sqrt(2 * Fi1 / odometry_accelerationAlpha);
        time_calculatedStage2 = 0;
    }
    else
    {
        //trapezni profil
        time_calculatedStage2 = (predznak * Fi_total - 2 * Fi1) / odometry_speedOmega;
    }

    current_time = current_time0 = sys_time;
    time_stage1 = current_time0 + time_calculatedStage1;
    time_stage2 = time_stage1 + time_calculatedStage2;
    time_stage3 = time_stage2 + time_calculatedStage3;
    ugao_ref = odometry_refrenceOrientation;
    dist_ref = odometry_refrenceDistance;

    robot_currentStatus = STATUS_MOVING;
    while(current_time < time_stage3)
        if(current_time != sys_time)
        {
            current_time = sys_time;
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

            if(current_time <= time_stage1)
            {
                w_ref += odometry_accelerationAlpha;
                odometry_refrenceSpeed += odometry_acceleration;
                delta = predznak * (w_ref - odometry_accelerationAlpha / 2);
                luk = predznak * R * delta / D_encoderWheel;
                ugao_ref += delta;
                dist_ref += robot_movingDirection * luk;
                odometry_refrenceOrientation = ugao_ref;
                odometry_refrenceDistance = dist_ref;
            }
            else if(current_time <= time_stage2)
            {
                w_ref = odometry_speedOmega;
                odometry_refrenceSpeed = odometry_speedMax;
                delta = predznak * odometry_speedOmega;
                luk = predznak * R * delta / D_encoderWheel;
                ugao_ref += delta;
                dist_ref += robot_movingDirection * luk;
                odometry_refrenceOrientation = ugao_ref;
                odometry_refrenceDistance = dist_ref;
            }
            else if(current_time <= time_stage3)
            {
                w_ref -= odometry_accelerationAlpha;
                odometry_refrenceSpeed -= odometry_acceleration;
                delta = predznak * (w_ref + odometry_accelerationAlpha / 2);
                luk = predznak * R * delta / D_encoderWheel;
                ugao_ref += delta;
                dist_ref += robot_movingDirection * luk;
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
    odometry_maxSpeedSet = tmp;
    setSpeedAccel(K2  * (unsigned char)odometry_maxSpeedSet / 256);
} // end of setSpeed(...)
