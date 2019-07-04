/*******************************************
 * library with the functions for the guts *
 * of the simulator                        *
 *******************************************/

#ifndef SIMULATOR_ACKERMAN_SIMULATION_H
#define SIMULATOR_ACKERMAN_SIMULATION_H

#ifdef __cplusplus
extern "C" {
#endif

//#define __USE_RL_CONTROL
#ifdef __USE_RL_CONTROL

void
set_rl_control(double steering, double throttle, double brake);

#endif

#include <carmen/simulator_ackerman.h>

/* recalculates the actual position */

void carmen_simulator_ackerman_recalc_pos(carmen_simulator_ackerman_config_t *simulator_config,
		int use_velocity_nn, int use_phi_nn, int connected_to_iron_bird, double iron_bird_v, double iron_bird_phi);

/* produce a laser message based upon the current
   position */

void carmen_simulator_ackerman_calc_laser_msg(carmen_laser_laser_message *flaser,
		carmen_simulator_ackerman_config_t *simulator_config, int is_rear_laser);

#ifdef __cplusplus
}
#endif

#endif
