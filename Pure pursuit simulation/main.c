#include"pure_pursuit.h"

int main(void) {
	initSim();
	while (1) {
		sim_counter++;
		if (sim_counter % 10000000 == 0) {

			updateSteering(sim_xp, sim_yp, sim_d);
			updateWaypoint(sim_xp, sim_yp);
			updatePursuit(sim_xp, sim_yp);
			updateRover();
			log_data();
		}
		
	}
}
