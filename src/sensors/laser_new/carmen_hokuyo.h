#ifndef CARMEN_HOKUYO_H
#define CARMEN_HOKUYO_H
#include "carmen_laser_device.h"

carmen_laser_device_t* carmen_create_hokuyo_instance(carmen_laser_laser_config_t* config, int laser_id);

int carmen_init_hokuyo_configs(void);

#endif

