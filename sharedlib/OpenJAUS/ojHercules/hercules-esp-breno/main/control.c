#include "control.h"

static const char* TAG = "CONTROL module";

void
motor_task (void* parameters)
{
    // Motor control task implementation
}

void
servo_task (void* parameters)
{
    // CAN task implementation using the global variables velocity and
    // velocityMutex
}