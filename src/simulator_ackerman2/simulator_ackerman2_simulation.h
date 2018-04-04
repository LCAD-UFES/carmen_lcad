/*******************************************
 * library with the functions for the guts *
 * of the simulator                        *
 *******************************************/

#ifndef SIMULATOR_ACKERMAN_SIMULATION_H
#define SIMULATOR_ACKERMAN_SIMULATION_H


#include <carmen/carmen.h>
#include "../simulator_ackerman/simulator_ackerman.h"


void carmen_simulator_ackerman2_recalc_pos(carmen_simulator_ackerman_config_t *simulator_config);


void carmen_simulator_ackerman2_calc_laser_msg(carmen_laser_laser_message *flaser,
		carmen_simulator_ackerman_config_t *simulator_config, int is_rear_laser);

#endif
