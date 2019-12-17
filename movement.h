#ifndef MOVEMENT_H
#define	MOVEMENT_H

#include "globals.h"
enum States
{
    STATUS_IDLE         = 'I',
    STATUS_MOVING       = 'M',
    STATUS_STUCK        = 'S',
    STATUS_ROTATING     = 'R',
    STATUS_ERROR        = 'E'
};

void robot_resetDriver(void);
void robot_setPosition(int X, int Y, int orientation);
void robot_returnInfo(void);
void odometry_setAcceleration(float v);
void robot_moveXY(int Xd, int Yd, unsigned char robot_maxSpeed, char robot_movingDirection);
void robot_moveLinear(int robot_distance, unsigned char robot_maxSpeed);
void robot_rotateAbsolute(int robot_orientation);
char robot_rotate(int robot_orientation);
void robot_arc(long Xc, long Yc, int Fi, char robot_movingDirection);
void robot_stop(void);
void robot_setSpeed(unsigned char tmp);
enum States robot_getStatus(void);
void robot_forceStatus(enum States);

#endif	/* MOVEMENT_H */
