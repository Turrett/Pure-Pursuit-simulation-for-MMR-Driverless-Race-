#include "pure_pursuit.h"

int main(void) {
	double sim_xp = 0.0;
	double sim_yp = 0.0;
	double sim_d = 0.0;
	int NUM_WP = 17;
	long sim_counter = 0;
	double sim_t = 0.0;
	double sim_v = 0.0;
	double sim_xv = 0.0;
	double sim_yv = 0.0;

	double steer_fx = 0.0;
	double steer_fy = 0.0;

	double way_pts[17][2] = { {130.0, 380.0}, {80.0, 250.0},{90.0,220.0}, {10.0,200.0},{80.0, 100.0},{100.0,90.0},{130.0,150.0},{150.0,200.0},{200.0,250.0},{250.0,130.0},{300.0,190.0},{350.0,100.0},{400.0,170.0},{450.0,300.0},{430.0,350.0},{300.0,470.0},{180.0,500.0} }; //qui inserisco i valori dei waypoints 
	//waypont di prova qua sotto 
	//{130.0, 380.0}, {80.0, 250.0},{90.0,220.0}, {10.0,200.0},{80.0, 100.0},{100.0,90.0},{130.0,150.0},{150.0,200.0},{200.0,250.0},{250.0,130.0},{300.0,190.0},{350.0,100.0},{400.0,170.0},{450.0,300.0},{430.0,350.0},{300.0,470.0},{180.0,500.0}
	int cur_way_pt = 0;
	initSim(&sim_xp , &sim_yp, &cur_way_pt , &steer_fx, &steer_fy, &sim_v, &sim_d, &sim_xv, &sim_yv);
	while (1) {
		sim_counter++;
		if (sim_counter % 10000000 == 0) {

			updateSteering(sim_xp, sim_yp, & sim_d,steer_fx,steer_fy);
			updateWaypoint(sim_xp, sim_yp,way_pts,&cur_way_pt,NUM_WP);
			updatePursuit(sim_xp, sim_yp,cur_way_pt,&steer_fx,&steer_fy,NUM_WP,way_pts);
			updateRover(sim_v,sim_d,&sim_xv,&sim_yv,&sim_xp,&sim_yp);
			log_data(sim_xp,sim_yp,cur_way_pt,steer_fx,steer_fy);
		}
		
	}
}
