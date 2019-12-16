#include    <p33FJ128MC802.h>

/* 
 * File:   globals.h
 * Author: mrmot
 *
 * Created on November 24, 2012, 3:25 PM
 */

#ifndef GLOBALS_H
#define	GLOBALS_H


/*
 1. Connect odometry cables to 5V, A, B, GND
 2. Left side cable to the left encoder, same for the right
 3. Mis-match sides with the encoders for the motors
 4. Left encoder should move left motor, same for the right.  
 -> if the left moves the right, swap motor connectors
 5. Moving forward encoder, the motor should move backward
 -> if the forward encoder moves the motor forward, swap the cables on the connector
 6. Check if everything works
 7. Measure d_tocka and D_tocka
 8. Calculate K1, K2
 9.  PWML = commande_distance
        PWMD = commande_distance
 10. Pid Gp_D & Gd_D
 11. Same for the rotation, just leave the commande_rotate in (dont forget -)
 12. Pid Gp_T & Gd_T
 13. Combine everything (original PWML, PWMD)
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

#define ROBOT_ACCEL_NUMBER 1500 // ovo se manje od zavisnosti koliko brzo da da gas robot (sto veci to sporije)

#ifdef BIG_ROBOT 
#define d_tocka	82.26           // precnik odometrijskog tocka odometrijskih
#define D_tocka	347.26          //rastojanje izmedju tockova odometrijskih
#define K1	34620               //(0.5 + 8*2048.0f * D_tocka / d_tocka)  // broj ikremenata po krugu
#define K2	15.873              //(long)(0.5 + 4*2048.0f / (d_tocka * PI))  //za konverziju mm u inkremente == 121.26

// napred/nazad P(I)D
#define Gp_D	6.6             
#define Gd_D	18.6

// rotacija P(I)D
#define Gp_T	3.25 
#define Gd_T	15
#endif



#ifdef SMALL_ROBOT
//42.4
#define d_tocka	65 //79.8            // precnik odometrijskog tocka
#define D_tocka	255//207.5           //rastojanje izmedju tockova
#define K1 	64276//41785               //(43841long)(0.5 + 8*2048.0f * D_tocka / d_tocka)  //broj ikremenata po krugu
#define K2	40.64//32.400              //(long)(0.5 + 4*2048.0f / (d_tocka * PI))  //za konverziju mm u inkremente == 121.26
#define Gp_D	2.25         
#define Gd_D	23.06       

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
