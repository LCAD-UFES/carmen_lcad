#ifndef TEST_H
#define TEST_H

#include "system.h"

void test_task(void *parameters);
void fake_odometry_task();
void fake_commands_task();
void fake_step_motor_task();
void measure_encoder_task();

#endif /* TEST_H */