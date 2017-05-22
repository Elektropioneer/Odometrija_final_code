#include    <p33FJ128MC802.h>

/* 
 * File:   globals.h
 * Author: mrmot
 *
 * Created on November 24, 2012, 3:25 PM
 */

#ifndef GLOBALS_H
#define	GLOBALS_H

#define BIG_ROBOT

#define PI	3.1415926535897932384626433832795

#ifdef BIG_ROBOT
#define d_tocka	82.26//51.75 // precnik odometrijskog tocka
#define D_tocka	347.26//348//348.76//167.5 //rastojanje izmedju tockova
#define K1	34620//34749//(43841long)(0.5 + 8*2048.0f * D_tocka / d_tocka)  //broj ikremenata po krugu
#define K2	15.873//15.853// 15.837//(long)(0.5 + 4*2048.0f / (d_tocka * PI))  //za konverziju mm u inkremente == 121.26
#define Gp_D	6.6//5.8
#define Gd_D	18.6//24

#define Gp_T	3.25 //4.6
#define Gd_T	15
#endif
#ifdef SMALL_ROBOT
//42.4
#define d_tocka	79.8//51.75 // precnik odometrijskog tocka
#define D_tocka	207.5//167.5 //rastojanje izmedju tockova
#define K1	41785//40850//21584//(43841long)(0.5 + 8*2048.0f * D_tocka / d_tocka)  //broj ikremenata po krugu
#define K2	32.400//16.139// 32.3//(long)(0.5 + 4*2048.0f / (d_tocka * PI))  //za konverziju mm u inkremente == 121.26
#define Gp_D	2.25         //2.2
#define Gd_D	23.06       //25.06

#define Gp_T	0.70
#define Gd_T	4.1
#endif

//makroi za zadavanje smera motora:
#define LEVI_NAPRED	LATBbits.LATB14=0; LATBbits.LATB12=1
#define LEVI_NAZAD	LATBbits.LATB14=1; LATBbits.LATB12=0
#define DESNI_NAPRED 	LATBbits.LATB11=1; LATBbits.LATB8=0
#define DESNI_NAZAD 	LATBbits.LATB11=0; LATBbits.LATB8=1

//POMOCNE PROMENLJIVE:
extern char brint;
extern long positionR, positionL;               //trenutne pozicije na enkoderima
extern int vR, vL, greska_pred, greska_pred_R;  //trenutne brzine na motorima
extern int zaglavL, zaglavR;                    //detekcija zaglavljivanja
extern unsigned char brzinaL;
extern long L, orientation, teta;
extern long long int Xlong, Ylong;
extern float vmax, accel;
extern float omega, alfa;
extern long X, Y;
extern long brojac,i;
extern unsigned long sys_time;
//PROMENLJIVE POTREBNE ZA REGULACIJU
extern int PWML, PWMD;
extern long t_ref, d_ref;
extern float v_ref;


#endif	/* GLOBALS_H */
