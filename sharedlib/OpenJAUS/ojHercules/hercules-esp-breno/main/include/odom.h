#ifndef ODOM_H
#define ODOM_H

#include "system.h"

void encoder_task(void *parameters);
void steering_reading(void *parameters);

#endif /* ODOM_H */