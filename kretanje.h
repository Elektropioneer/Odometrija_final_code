#ifndef KRETANJE_H
#define	KRETANJE_H

#include "globals.h"
enum States
{
    STATUS_IDLE         = 'I',
    STATUS_MOVING       = 'M',
    STATUS_STUCK        = 'S',
    STATUS_ROTATING     = 'R',
    STATUS_ERROR        = 'E'
};

void resetDriver(void);
void setPosition(int X, int Y, int orientation);
void sendStatusAndPosition(void);
void setSpeedAccel(float v);
void gotoXY(int Xd, int Yd, unsigned char robot_maxSpeed, char robot_movingDirection);
void kretanje_pravo(int robot_distance, unsigned char robot_maxSpeed);
void apsolutni_ugao(int robot_orientation);
char okret(int robot_orientation);
void kurva(long Xc, long Yc, int Fi, char robot_movingDirection);
void stop(void);
void setSpeed(unsigned char tmp);
enum States getStatus(void);
void forceStatus(enum States);

#endif	/* KRETANJE_H */
