#ifndef CAN_H
#define CAN_H

#include "system.h"

int can_setup ();
void can_reading_task (void* parameters);
void can_writing_task (void* parameters);

#endif /* CAN_H */
