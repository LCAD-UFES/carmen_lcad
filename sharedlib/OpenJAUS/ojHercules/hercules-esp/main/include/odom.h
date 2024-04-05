#ifndef ODOM_H
#define ODOM_H

#include "system.h"

void right_encoder_task( void );
void left_encoder_task( void );
void steering_reading_task( void );
bool adc_calibration_init(void);

#endif /* ODOM_H */