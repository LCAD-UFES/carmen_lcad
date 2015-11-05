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

carmen_FILE *logfile = NULL;
carmen_logfile_index_p logfile_index = NULL;

int main(int argc  __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
  int i;
  char line[100001];

  carmen_point_t last_odom   = {MAXDOUBLE, MAXDOUBLE, MAXDOUBLE};
  carmen_point_t last_laser  = {MAXDOUBLE, MAXDOUBLE, MAXDOUBLE};

  double odom_dist  = 0;
  double laser_dist = 0;
  double starttime  = -1;
  double lasttimestamp = 0;

  carmen_base_odometry_message odometry;
  carmen_erase_structure(&odometry, sizeof(odometry) );

  carmen_robot_laser_message laser;
  carmen_erase_structure(&laser, sizeof(laser) );

  /* open the logfile */
  logfile = carmen_fopen(argv[1], "r");
  if(logfile == NULL)
    carmen_die("Error: could not open file %s for reading.\n", argv[1]);

  /* index the logfile */
  logfile_index = carmen_logfile_index_messages(logfile);

  for(i = 0; i < logfile_index->num_messages; i++) {
    /* read i-th line */
    carmen_logfile_read_line(logfile_index, logfile, i, 100000, line);

    /* read odometry message */
    if(strncmp(line, "ODOM ", 5) == 0) {
      carmen_string_to_base_odometry_message(carmen_next_word(line), 
					     &odometry);


      fprintf(stderr,"O");
      if (last_odom.x != MAXDOUBLE) {
	odom_dist += hypot(last_odom.x - odometry.x, last_odom.y - odometry.y);
      }
      last_odom.x = odometry.x;
      last_odom.y = odometry.y;
      last_odom.theta = odometry.theta;

      if (starttime < 0)
	starttime = odometry.timestamp;
      lasttimestamp = odometry.timestamp;

    }
    else if(strncmp(line, "ROBOTLASER0 ", 12) == 0) {
      carmen_string_to_robot_laser_message(carmen_next_word(line), 
					   &laser);

      fprintf(stderr,"L");
      if (last_laser.x != MAXDOUBLE) {
	laser_dist += hypot(last_laser.x - laser.laser_pose.x,
			    last_laser.y - laser.laser_pose.y);
      }
      last_laser = laser.laser_pose;

      if (starttime < 0)
	starttime = laser.timestamp;
      lasttimestamp = laser.timestamp;


    }
    else if(strncmp(line, "FLASER ", 7) == 0) {
      carmen_string_to_robot_laser_message_orig(carmen_next_word(line), 
						&laser);

      fprintf(stderr,"l");
      if (last_laser.x != MAXDOUBLE) {
	laser_dist += hypot(last_laser.x - laser.laser_pose.x,last_laser.y - laser.laser_pose.y);
      }
      last_laser = laser.laser_pose;
      
      if (starttime <0) {
	starttime = laser.timestamp;
      fprintf(stderr,"%s\n",line);

      }
      lasttimestamp = laser.timestamp;
    }
  }

  fprintf(stderr,"%s\n",line);

  fprintf(stderr, "Time                         : %.2f s\n", lasttimestamp-starttime);
  fprintf(stderr, "Traveled distance (odometry) : %.2f m   =>  %.2f m/s\n", odom_dist, odom_dist/(lasttimestamp-starttime));
  fprintf(stderr, "Traveled distance (laser)    : %.2f m   =>  %.2f m/s\n", laser_dist, laser_dist/(lasttimestamp-starttime));
  return 0;
}
