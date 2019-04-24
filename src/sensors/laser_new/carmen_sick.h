#ifndef CARMEN_SICK_H
#define CARMEN_SICK_H
#include "carmen_laser_device.h"

carmen_laser_device_t* carmen_create_sick_instance(carmen_laser_laser_config_t* config, int laser_id);

int carmen_init_sick_configs(void);

#endif

