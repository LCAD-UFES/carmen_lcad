#ifndef CONTROL_H
#define CONTROL_H

#include "system.h"

void motor_task( void );
void servo_task( void );
void step_motor_task ( void );
void config_step_motor_pins( void );

#endif /* CONTROL_H */