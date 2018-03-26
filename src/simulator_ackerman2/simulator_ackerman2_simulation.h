/*******************************************
 * library with the functions for the guts *
 * of the simulator                        *
 *******************************************/

#ifndef SIMULATOR_ACKERMAN_SIMULATION_H
#define SIMULATOR_ACKERMAN_SIMULATION_H

//#define __USE_RL_CONTROL
#ifdef __USE_RL_CONTROL

void
set_rl_control(double steering, double throttle, double brake);

#endif


void carmen_simulator_ackerman_recalc_pos(carmen_simulator_ackerman_config_t *simulator_config);


void carmen_simulator_ackerman_calc_laser_msg(carmen_laser_laser_message *flaser,
		carmen_simulator_ackerman_config_t *simulator_config, int is_rear_laser);

#endif
