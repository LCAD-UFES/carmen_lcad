#ifndef CAN_H
#define CAN_H

#include "system.h"

int can_setup ( void );
void can_reading_task ( void) ;
void can_writing_task ( void );

#endif /* CAN_H */
