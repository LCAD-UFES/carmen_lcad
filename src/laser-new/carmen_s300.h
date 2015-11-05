#ifndef CARMEN_S300_H
#define CARMEN_S300_H
#include "carmen_laser_device.h"

carmen_laser_device_t* carmen_create_s300_instance(carmen_laser_laser_config_t* config, int laser_id);

int carmen_init_s300_configs(void);

#endif
