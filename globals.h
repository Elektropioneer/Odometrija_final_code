#include    <p33FJ128MC802.h>

#ifndef GLOBALS_H
#define GLOBALS_H


/*
 1. Connect odometry cables to 5V, A, B, GND
 2. Left side cable to the left encoder, same for the right
 3. Mis-match sides with the encoders for the motors
 4. Left encoder should move left motor, same for the right.  
 -> if the left moves the right, swap motor connectors
 5. Moving forward encoder, the motor should move backward
 -> if the forward encoder moves the motor forward, swap the cables on the connector
 6. Check if everything works
 7. Measure d_encoderWheel and D_encoderWheel
 8. Calculate K1, K2
 9.   = odometry_regulatorDistance
        motor_currentRightPWM = odometry_regulatorDistance
 10. Pid regulator_distanceP & regulator_distanceD
 11. Same for the rotation, just leave the commande_rotate in (dont forget -)
 12. Pid regulator_rotationP & regulator_rotationD
 13. Combine everything (original , motor_currentRightPWM)
 14. Go back to 9. and reconfigure until you go fucking insane
 15. Add kretanje_pravo for 1000 and -1000. 
 -> K2 (K1) to adjust for 1000mm. 
 -> check if the robots sweeps, if yes fix with rollers and weight
 16. Remove kretanje_pravo, and put okret(360) no scope
 -> Tune K1 (K2) to adjust for 360 no scope
 -> check if the robots sweeps, if yes fix with rollers and weight
 17. Test cube motion
 -> if it is not percise go to 15. 
 
 * Extra fancy
 -> communication every command
 -> connect communication and test hard stop (for detection). For testing if it loses traction/position
 
 
 * Tips
 -> dont use hard stop on high speed pursuits
 -> use set position as often as possible 
 -> linear motion no kurve
 
 */




//#define BIG_ROBOT
#define SMALL_ROBOT



#define PI	3.1415926535897932384626433832795



// misc defines (global)
#define odometry_accelerationParameter           1500                        // the acceleration parameter, the higher the number it slower accelerates
#define motor_saturationPWMValue                 3200                        // max/min PWM value
#define odometry_shortDistance                   500                         // a short distance for acceleration modification
#define odometry_accelerationShorterDistance     K2/3                        // in gotoXY/kretanje_pravo if there is a distance <500mm it will accelerate with this number
#define odometry_accelerationKurva               K2 / 32                     // the max acceleration the kurva function will do


#ifdef BIG_ROBOT 
#define d_encoderWheel	82.26            // precnik odometrijskog tocka odometrijskih
#define D_encoderWheel	347.26           // rastojanje izmedju tockova odometrijskih

// calculate with (0.5 + 8*2048 * D_encoderWheel / d_encoderWheel) - used for rotation calib
#define K1	34620               
// calculate with (0.5 + 4*2048.0f / (d_encoderWheel * PI)) - used for distance calib
#define K2	15.873             

// napred/nazad P(I)D
#define regulator_distanceP	6.6             
#define regulator_distanceD	18.6

// rotacija P(I)D
#define regulator_rotationP	3.25 
#define regulator_rotationD	15

#endif



#ifdef SMALL_ROBOT

#define d_encoderWheel	65                   // precnik odometrijskog tocka
#define D_encoderWheel	255                  // rastojanje izmedju tockova

// calculate with (0.5 + 8*2048 * D_encoderWheel / d_encoderWheel) - used for rotation calib
#define K1	34620               
// calculate with (0.5 + 4*2048.0f / (d_encoderWheel * PI)) - used for distance calib
#define K2	15.873      

#define regulator_distanceP	2.25         
#define regulator_distanceD	23.06       

#define regulator_rotationP	0.70
#define regulator_rotationD	4.1

#endif




//POMOCNE PROMENLJIVE:
extern char odometry_loop_counter;
extern long encoder_rightIncrements, encoder_leftIncrements;               
extern int encoder_rightCurrentIncrements, encoder_leftCurrentIncrements, greska_pred, greska_pred_R;  
extern int odometry_stuckDistance, odometry_stuckOrientation;                    
extern unsigned char odometry_maxSpeedSet;
extern long odometry_incrementsDistance, odometry_incrementsOrientation, odometry_orientationTeta;
extern long long int odometry_incrementsX, odometry_incrementsY;
extern float odometry_speedMax, odometry_acceleration;
extern float odometry_speedOmega, odometry_accelerationAlpha;
extern long odometry_milliX, odometry_milliY;
extern unsigned long sys_time;
//PROMENLJIVE POTREBNE ZA REGULACIJU
extern int motor_currentLeftPWM, motor_currentRightPWM;
extern long odometry_refrenceOrientation, odometry_refrenceDistance;
extern float odometry_refrenceSpeed;


#endif	/* GLOBALS_H */
