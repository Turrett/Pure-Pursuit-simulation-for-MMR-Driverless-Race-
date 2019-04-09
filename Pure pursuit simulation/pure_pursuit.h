#pragma once
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
//setup dell' ambiente 
#define STEP_TIME 100;
#define SCALE_P 1.0
#define SCALE_V 8.0
#define WP_DIST 20.0 //più il valore è piccolo più il puntatore dovrà avvicinarsi al waypoint per passare al prossimo 
#define FOLLOW_DIST 50.0 // più il valore è piccolo più il puntatore avrà difficoltà a raggiungere il navigatore 
#define X_OFF 0;//due offset per traslare i waypoint e il navigatore   
#define Y_OFF 0
#define CRUMB_SZ 500
#define X 0
#define Y 1
#define DIM 2


/*-------------------------------------------------------------------------------------*/

extern void setVelocity(double val, double *sim_v);

extern void setSteering(double val, double *sim_d);

extern void setFollow(double x, double y, double *set_fx, double *set_fy);

extern void setPosition(double x, double y, double *sim_xp, double *sim_yp);

extern void changeSteeringBy(double delta, double *sim_d);

extern int getPreviousWaypoint(double cur_way_pt, int NUM_WP);

extern double calcDist(double x, double y);

/*-----------------------------------------------------------------------------------*/

extern void initSim(double *sim_xp, double *sim_yp, int *cur_way_pt, double* steer_fx, double * steer_fy, double *sim_v, double *sim_d, double *sim_xv, double *sim_yv);

extern void updateSteering(double x, double y, double *dir, double steer_fx, double steer_fy);

extern void updateWaypoint(double x, double y, double way_pts[][2], int *cur_way_pt, int NUM_WP);

extern void updatePursuit(double x, double y, double cur_way_pt, double *steer_fx, double *steer_fy, int NUM_WP, double way_pts[][2]);

extern void updateRover(const double sim_v, const double sim_d, double *sim_xv, double *sim_yv, double *sim_xp, double *sim_yp);

extern void log_data(const double sim_xp, const double sim_yp, int cur_way_pt,const double steer_fx,const double steer_fy);




