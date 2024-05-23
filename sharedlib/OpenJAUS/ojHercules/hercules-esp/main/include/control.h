#ifndef CONTROL_H
#define CONTROL_H

#include "system.h"

void motor_task();
void servo_task();
void step_motor_task ();
void config_step_motor_pins();

#endif /* CONTROL_H */