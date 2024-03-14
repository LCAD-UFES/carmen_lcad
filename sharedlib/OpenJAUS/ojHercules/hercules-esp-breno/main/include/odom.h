#ifndef ODOM_H
#define ODOM_H

#include "system.h"

void right_encoder_task(void *parameters);
void left_encoder_task(void *parameters);
void steering_reading(void *parameters);

#endif /* ODOM_H */