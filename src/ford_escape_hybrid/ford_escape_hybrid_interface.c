/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include "ford_escape_hybrid_messages.h"

void
carmen_ford_escape_subscribe_status_message(carmen_ford_escape_status_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_FORD_ESCAPE_STATUS_NAME,
		  	  	  	  	  CARMEN_FORD_ESCAPE_STATUS_FMT,
                           message, sizeof(carmen_ford_escape_status_message),
                           handler, subscribe_how);
}


void
carmen_ford_escape_unsubscribe_status_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_FORD_ESCAPE_STATUS_NAME, handler);
}

