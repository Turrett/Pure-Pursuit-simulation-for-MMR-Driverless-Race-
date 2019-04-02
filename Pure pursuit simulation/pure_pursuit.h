#pragma once
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
//setup dell' ambiente 
double STEP_TIME = 100;
double SCALE_P = 1.0;
double SCALE_V = 8.0;
double WP_DIST = 20.0;//più il valore è piccolo più il puntatore dovrà avvicinarsi al waypoint per passare al prossimo 
double FOLLOW_DIST = 50.0;// più il valore è piccolo più il puntatore avrà difficoltà a raggiungere il navigatore 
int X_OFF = 0;//due offset per traslare i waypoint e il navigatore   
int Y_OFF = 0;
int CRUMB_SZ = 500;
int NUM_WP = 17;//numero di punti , devo sempre inserire il numero esatto di punti contuti in way_pts se no crasha
int X = 0;
int Y = 1;
int DIM = 2;

bool isStart = true ;

long sim_counter = 0;
double sim_t = 0.0;
double sim_v = 0.0;
double sim_d = 0.0;
double sim_xp = 0.0;
double sim_yp = 0.0;
double sim_xv = 0.0;
double sim_yv = 0.0;

double steer_fx = 0.0;
double steer_fy = 0.0;

double way_pts[17][2] = { {130.0, 380.0}, {80.0, 250.0},{90.0,220.0}, {10.0,200.0},{80.0, 100.0},{100.0,90.0},{130.0,150.0},{150.0,200.0},{200.0,250.0},{250.0,130.0},{300.0,190.0},{350.0,100.0},{400.0,170.0},{450.0,300.0},{430.0,350.0},{300.0,470.0},{180.0,500.0} }; //qui inserisco i valori dei waypoints 
int cur_way_pt = 0;

extern void setPosition(double x, double y);
extern void setVelocity(double val);
extern void setSteering(double val);
extern void changeSteeringBy(double delta);
extern void setFollow(double x, double y);
extern int getCurrentWaypoint();
extern int getPreviousWaypoint();


extern void initSim();

extern void updateSteering(double x, double y, double dir);

extern void updateWaypoint(double x, double y);

extern void updatePursuit(double x, double y);

extern void updateRover();

extern double calcDist(double x, double y);

extern void log_data();

/*-----------------------------------------------------------------------------------*/

void setPosition(double x, double y) {
	sim_xp = x;
	sim_yp = y;
}

void setVelocity(double val) {
	sim_v = val;
}

void setSteering(double val) {
	sim_d = val;
}

void changeSteeringBy(double delta) {
	sim_d += delta;
}

void setFollow(double x, double y) {
	steer_fx = x;
	steer_fy = y;
}

int getCurrentWaypoint() {
	return cur_way_pt;
}

int getPreviousWaypoint() {
	int pwp = (cur_way_pt == 0) ? (NUM_WP - 1) : cur_way_pt - 1;
	return pwp;
}

/*--------------------------------------------------------------------------------*/

void initSim() {
	cur_way_pt = 1;
	setFollow(280.0, 500);
	setPosition(300.0, 400.0);//punto di inizio percorso 
	setVelocity(1.5);//velocità , più lento è meglio 
	setSteering(4);//sterzata 
	updateRover();
}

void updateSteering(double x, double y, double dir) {
	double fdx = (x - steer_fx);
	double fdy = (y - steer_fy);
	double fdist = calcDist(fdx, fdy);

	// Compute the cross product -- tells which way to turn
	double cdir = cos((float)dir);
	double sdir = sin((float)dir);
	double xprod = (sdir*fdx - cdir * fdy) / fdist;

	double val = 0.0;
	if (xprod < -0.1) 
		val = 0.06;
	else if (xprod > 0.1)
		val = -0.06;
	changeSteeringBy(-val);
}

void updateWaypoint(double x, double y) {
	double cwpx = way_pts[cur_way_pt][X];
	double cwpy = way_pts[cur_way_pt][Y];
	double cdist = calcDist(x - cwpx, y - cwpy);

	if (cdist < WP_DIST) cur_way_pt++;
	if (cur_way_pt >= NUM_WP) cur_way_pt = 0;
}

void updatePursuit(double x, double y) {
	double fdist = calcDist(x - steer_fx, y - steer_fy);
	if (fdist < FOLLOW_DIST) {
		int cwp = getCurrentWaypoint();
		int pwp = getPreviousWaypoint();
		double wpx = way_pts[cwp][X] - way_pts[pwp][X];
		double wpy = way_pts[cwp][Y] - way_pts[pwp][Y];
		double wpdist = calcDist(wpx, wpy);
		double cdist = calcDist(x - way_pts[cwp][X], y - way_pts[cwp][Y]);

		double wp_ratio = (wpdist - cdist + FOLLOW_DIST) / wpdist;
		if (wp_ratio > 1.0) wp_ratio = 1.0;

		double nx = way_pts[pwp][X] + wp_ratio * wpx;
		double ny = way_pts[pwp][Y] + wp_ratio * wpy;
		setFollow(nx, ny);
	}
}

void updateRover() {
	sim_xv = sim_v * cos((float)sim_d);
	sim_yv = sim_v * sin((float)sim_d);
	sim_xp += sim_xv;
	sim_yp += sim_yv;
}


double calcDist(double x, double y) {
	double d = sqrt((float)(x*x + y * y));
	return d;
}

void log_data() {
	int way = getCurrentWaypoint();
	printf("X_POSITION_:%lf   Y_POSITION_:%lf   CURRENT_WAYPOINT_FOLLOWING:%d \n", sim_xp , sim_yp , way);
}



/*void keyPressed() {
	switch (keyPressed) {
	case ' ':
		initSim();
		break;
	case 'p':
		isStart = !isStart;
		break;
	}
}*/


