#include "pure_pursuit.h"

void setVelocity(double val, double *sim_v) {//
	*sim_v = val;
}

void setPosition(double x, double y, double *sim_xp, double *sim_yp) {
	*sim_xp = x;
	*sim_yp = y;
}

void setSteering(double val, double *sim_d) {//
	*sim_d = val;
}

void setFollow(double x, double y, double *set_fx, double *set_fy) {//
	*set_fx = x;
	*set_fy = y;

}


void changeSteeringBy(double delta, double *sim_d) {//
	*sim_d += delta;
}

int getPreviousWaypoint(double cur_way_pt, int NUM_WP) {//
	int pwp = (cur_way_pt == 0) ? (NUM_WP - 1) : cur_way_pt - 1;
	return pwp;
}

double calcDist(double x, double y) {//
	double d = sqrt((float)(x*x + y * y));
	return d;
}
/*-----------------------------------------------------------------------*/
void initSim(double *sim_xp, double *sim_yp, int *cur_way_pt, double* steer_fx, double * steer_fy, double *sim_v, double *sim_d, double *sim_xv,double *sim_yv) {
	*cur_way_pt = 0;
	setPosition(200, 300, sim_xp, sim_yp);
	setFollow(300, 400, steer_fx, steer_fy);
	setVelocity(1.5,sim_v);//velocità , più lento è meglio 
	setSteering(1,sim_d);//direzione della macchina 
	updateRover(*sim_v, *sim_d, sim_xv, sim_yv,sim_xp, sim_yp);
}

void updateSteering(double x, double y, double *dir, double steer_fx, double steer_fy) {// calcola la distanza fra la macchina e il navigatore
	double fdx = (x - steer_fx);
	double fdy = (y - steer_fy);
	double fdist = calcDist(fdx, fdy);

	// calcola (seno direzione macchina * distanza x dal navigatore - coseno direzione macchina * distanza y dal navigatore ) / distanza fra il navigatore e la macchina  
	//e ne modifica la direzione di conseguenza 
	double cdir = cos((float)*dir);
	double sdir = sin((float)*dir);
	double xprod = (sdir*fdx - cdir * fdy) / fdist;

	double val = 0.0;
	if (xprod < -0.1)
		val = 0.06;
	else if (xprod > 0.1)
		val = -0.06;
	changeSteeringBy(-val, dir);
}

void updateWaypoint(double x, double y, double way_pts[][2], int *cur_way_pt, int NUM_WP) {
	double cwpx = way_pts[*cur_way_pt][X];
	double cwpy = way_pts[*cur_way_pt][Y];
	double cdist = calcDist(x - cwpx, y - cwpy);

	if (cdist < WP_DIST) (*cur_way_pt)++;
	if ((*cur_way_pt) >= NUM_WP) *cur_way_pt = 0;
}

void updatePursuit(double x, double y, double cur_way_pt, double *steer_fx, double *steer_fy, int NUM_WP, double way_pts[][2]) {
	double fdist = calcDist(x - (*steer_fx), y - (*steer_fy));
	if (fdist < FOLLOW_DIST) {
		int cwp = cur_way_pt;
		int pwp = getPreviousWaypoint(cur_way_pt, NUM_WP);
		double wpx = way_pts[cwp][X] - way_pts[pwp][X];
		double wpy = way_pts[cwp][Y] - way_pts[pwp][Y];
		double wpdist = calcDist(wpx, wpy);
		double cdist = calcDist(x - way_pts[cwp][X], y - way_pts[cwp][Y]);

		double wp_ratio = (wpdist - cdist + FOLLOW_DIST) / wpdist;
		if (wp_ratio > 1.0) wp_ratio = 1.0;

		double nx = way_pts[pwp][X] + wp_ratio * wpx;
		double ny = way_pts[pwp][Y] + wp_ratio * wpy;
		setFollow(nx, ny, steer_fx, steer_fy);
	}
}

void updateRover(const double sim_v, const double sim_d, double *sim_xv, double *sim_yv, double *sim_xp, double *sim_yp) {
	*sim_xv = sim_v * cos((float)sim_d);
	*sim_yv = sim_v * sin((float)sim_d);
	*sim_xp += *sim_xv;
	*sim_yp += *sim_yv;
}


void log_data(const double sim_xp, const double sim_yp, int cur_way_pt,const double steer_fx ,const double steer_fy) {
	int way = cur_way_pt;
	printf("X_POS_:%lf   Y_POS_:%lf   CUR_WAY_FOLLOW:%d  NAV_X:%lf  NAV_Y:%lf\n", sim_xp, sim_yp, way,steer_fx,steer_fy);
}
