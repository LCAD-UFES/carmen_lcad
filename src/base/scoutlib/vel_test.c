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
#include "Nclient.h"

#define        METRES_PER_INCH        0.0254
#define        METRES_TO_SCOUT        (METRES_PER_INCH/10.0)

#define        WHEELBASE              (13.4 * METRES_PER_INCH)
#define        ROT_VEL_FACT_RAD       (WHEELBASE/METRES_TO_SCOUT)

int main(int argc, char **argv)
{
  float vel, acc;

  if(argc < 3)
    carmen_die("usage: %s <rotation speed> <acceleration>\n", argv[0]);

  vel = 0.5 * carmen_degrees_to_radians(atof(argv[1])) * ROT_VEL_FACT_RAD;
  acc = atof(argv[2])/METRES_TO_SCOUT;

  connect_robot(1, MODEL_SCOUT2, "/dev/ttyS0", 38400);
  ac(acc, acc, 0);

  carmen_warn("rv: %f makes vel: %f\n", carmen_degrees_to_radians(atof(argv[1])), vel);

  while(1) {
    vm(vel, -vel, 0);
    usleep(250000);
  }
  return 0;
}
