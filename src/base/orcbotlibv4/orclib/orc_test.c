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
#include "../base_low_level.h"
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

int carmen_base_query_low_level(double *left_disp, double *right_disp,
				double *delta_time);

void carmen_base_command_velocity(double desired_velocity, 
				  double current_velocity,
				  unsigned char WHICH_MOTOR);

int main(int argc, char *argv[])
{
  struct stat buf;
  double x, y, theta, tv, rv;  
  double left_disp, right_disp, delta_time;
  int fd;

  if (argc != 2 || carmen_strncasecmp(argv[1], "-h", 2) == 0 ||
      carmen_strncasecmp(argv[1], "--h", 3) == 0) {
    if (argc > 2) 
      carmen_warn("Too many arguments.\n\n");
    else if (argc < 2)
      carmen_warn("Not enough arguments.\n\n");
    carmen_die("Usage: %s <serial dev name>\n", argv[0]);
  }
  
  // Does the device exist?

  carmen_warn("Device exists test: ");
  if (stat(argv[1], &buf) == 0) 
    carmen_warn("OK\n");
  else {
    carmen_warn("FAILED\n");
    carmen_die_syserror("Device exists test");
  }

  // Can we open the device?

  carmen_warn("Device open test: ");
  fd = open(argv[1], O_RDWR | O_SYNC | O_NOCTTY, 0);
  if (fd >= 0)
    carmen_warn("OK\n");
  else {
    carmen_warn("FAILED\n");
    carmen_die_syserror("Device open test");
  }
  //  close(fd);

  // Can we talk to the orc board properly? 

  carmen_warn("Orc recognized test: ");
  if (carmen_base_direct_initialize_robot("orc", argv[1]) == 0)
    carmen_warn("OK\n");
  else {
    carmen_die("FAILED\n");
  }

  // Can we move the wheels?  

  carmen_warn("Orc drive test: %sMAKE SURE THE ROBOT IS ON BLOCKS%s.\n",
	      carmen_red_code, carmen_normal_code);
  carmen_warn("Hit return to start the orc drive test...."); 
  scanf("%*c");

  if (1) {
  // Move left wheel 
  carmen_warn("Left wheel forwards test: ");
  carmen_base_command_velocity(1, 0, 0);
  sleep(1);
  carmen_base_command_velocity(0, 0, 0);
  sleep(1);
  carmen_base_query_low_level(&left_disp, &right_disp, &delta_time);
  //  carmen_warn("%f %f %f\n", left_disp, right_disp, delta_time);
  if (0 && fabs(right_disp) > 1e-3) 
    carmen_die("FAILED\nRight encoder moved. The motors need "
	       "to be swapped.\n");
  if (left_disp < -1e-3)
    carmen_die("FAILED\n Left encoder moved backwards. The motor is "
	       "upside-down.\n");
  if (fabs(left_disp) < 1e-3) 
    carmen_die("FAILED\nEncoder failed to move. Did the wheel move?\n");

  carmen_warn("OK\n");

  carmen_warn("Left wheel backwards test: ");
  carmen_base_command_velocity(-1, 0, 0);
  sleep(1);
  carmen_base_command_velocity(0, 0, 0);
  sleep(1);
  carmen_base_query_low_level(&left_disp, &right_disp, &delta_time);
  //  carmen_warn("%f %f %f\n", left_disp, right_disp, delta_time);
  if (0 && fabs(right_disp) > 1e-3) 
    carmen_die("FAILED\nRight encoder moved. Very odd.\n");
  if (left_disp > 1e-3)
    carmen_die("FAILED\n Left encoder moved forwards. Very odd.\n");
  if (fabs(left_disp) < 1e-3) 
    carmen_die("FAILED\nEncoder failed to move. Did the wheel move?\n");

  carmen_warn("OK\n");

  // Move right wheel

  carmen_warn("Right wheel forwards test: ");
  carmen_base_command_velocity(1, 0, 2);
  sleep(1);
  carmen_base_command_velocity(0, 0, 2);
  sleep(1);
  carmen_base_query_low_level(&left_disp, &right_disp, &delta_time);
  //  carmen_warn("%f %f %f\n", left_disp, right_disp, delta_time);
  if (0 && fabs(left_disp) > 1e-3) 
    carmen_die("FAILED\nLeft encoder moved. This is very odd, since it"
	       "also moved with\nthe other wheel.\n");
  if (right_disp < -1e-3)
    carmen_die("FAILED\nRight encoder moved backwards. The motor is "
	       "upside-down.\n");
  if (fabs(right_disp) < 1e-3) 
    carmen_die("FAILED\nEncoder failed to move. Did the wheel move?\n");
  carmen_warn("OK\n");
  
  carmen_warn("Right wheel backwards test: ");
  carmen_base_command_velocity(-1, 0, 2);
  sleep(1);
  carmen_base_command_velocity(0, 0, 2);
  sleep(1);
  carmen_base_query_low_level(&left_disp, &right_disp, &delta_time);
  //  carmen_warn("%f %f %f\n", left_disp, right_disp, delta_time);
  if (0 && fabs(left_disp) > 1e-3) 
    carmen_die("FAILED\nLeft encoder moved. Very odd.\n");
  if (right_disp > 1e-3)
    carmen_die("FAILED\nRight encoder moved forwards. Very odd.\n");
  if (fabs(right_disp) < 1e-3) 
    carmen_die("FAILED\nEncoder failed to move. Did the wheel move?\n");
  }
  carmen_warn("OK\n");

  carmen_base_direct_reset();

  carmen_warn("Orc drive test: ");
  if (carmen_base_direct_set_velocity(1.0, 0.0) == 0)
    carmen_warn("OK\n");
  else 
    carmen_die("FAILED\n");  

  // Let some encoder data build up

  carmen_warn("Sleep\n");
  sleep(1); 
  
  carmen_base_direct_set_velocity(0.0, 0.0);

  // Are we getting encoder data?

  carmen_warn("Orc encoder test: ");
  if (carmen_base_direct_get_integrated_state(&x, &y, &theta, &tv, &rv) == 0)
    carmen_warn("OK\n");
  else
    carmen_die("FAILED\n");
  
  // Is encoder data valid?

  carmen_warn("Orc encoder data validity test: ");
  if (fabs(x) > 0 && fabs(y) < 1e-2 && 
      fabs(carmen_radians_to_degrees(theta)) < 10) 
    carmen_warn("OK (%.2f)\n", x);
  else
    carmen_die("FAILED (%.2f %.2f %.2f)\n", x, y, 
	       carmen_radians_to_degrees(theta));

  carmen_base_direct_shutdown_robot();

  return 0;
}
