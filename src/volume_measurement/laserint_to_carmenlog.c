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
#include "writelog.h"

#define LINE_SIZE 30000
char line[LINE_SIZE], line2[LINE_SIZE];

carmen_base_odometry_message odometry;
double odometry_timestamp;
int current_odometry = 0;

carmen_robot_laser_message front_laser;
double front_laser_timestamp;
int current_front_laser = 0;

void get_all_params(carmen_FILE *outfile)
{
  char **variables, **values;
  int list_length;
  int index;

  char *robot_name;
  char **modules;
  int num_modules;
  int module_index;
  char *hostname;

  robot_name = carmen_param_get_robot();
  carmen_param_get_modules(&modules, &num_modules);

  carmen_logwrite_write_robot_name(robot_name, outfile);
  free(robot_name);

  carmen_param_get_paramserver_host(&hostname);
  for (module_index = 0; module_index < num_modules; module_index++) {
    if(carmen_param_get_all(modules[module_index], &variables, &values, NULL,
			    &list_length) < 0) {
      IPC_perror("Error retrieving all variables of module");
      exit(-1);
    }
    for(index = 0; index < list_length; index++) {
      carmen_logwrite_write_param(modules[module_index], variables[index], 
				values[index], carmen_get_time(), 
				hostname, outfile, carmen_get_time());
      free(variables[index]);
      free(values[index]);
    }
    free(variables);
    free(values);
    free(modules[module_index]);
  }
  free(hostname);
  free(modules);
}

int main(int argc, char **argv)
{
  FILE *fp;
  carmen_FILE *outfile;
  int m, d, y, h, min, temp1, temp2, i;
  float s;
//  int type_id;
  double timestamp, first_timestamp = 0;
  double last_front_laser_timestamp, last_odometry_timestamp;
  int first = 1;
  char key;

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);	

  if(argc < 3) {
    fprintf(stderr, "Error: wrong number of arguments.\n");
    fprintf(stderr, "Usage: %s laserint-logname logname\n", argv[0]);
    exit(1);
  }
  fp = fopen(argv[1], "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n", argv[1]);
    exit(1);
  }
	outfile = carmen_fopen(argv[2], "r");
	if (outfile != NULL) {
		fprintf(stderr, "Overwrite %s? ", argv[2]);
		scanf("%c", &key);
		if (toupper(key) != 'Y')
			exit(-1);
		carmen_fclose(outfile);
	}

  outfile = carmen_fopen(argv[2], "w");
  if(outfile == NULL) {
    fprintf(stderr, "Error: could not open file %s for writing.\n", argv[2]);
    exit(1);
  }

	front_laser.num_readings = 180;
	front_laser.range = (float *)calloc(180, sizeof(float));
	carmen_test_alloc(front_laser.range);

	front_laser.host = carmen_get_host();
	odometry.host = carmen_get_host();

	carmen_logwrite_write_header(outfile);
	carmen_logwrite_write_robot_name("beesoft", outfile);
  get_all_params(outfile);

  while(!feof(fp)) {
    fgets(line, LINE_SIZE, fp);
    if(strncmp(line, "@SENS", 5) == 0) {

      fgets(line2, 7, fp);

      if(strncmp(line2, "#LASER", 6) == 0) {
				last_front_laser_timestamp = front_laser_timestamp;
	
				sscanf(line, "@SENS %d-%d-%d %d:%d:%f\n", &m, &d, &y, &h, &min, &s);
				timestamp = h * 3600.0 + min * 60.0 + s;
				fscanf(fp, " %d %d: ", &temp1, &temp2);
				for(i = 0; i < 180; i++) {
					fscanf(fp, "%d", &temp1);
					front_laser.range[i] = temp1/100.0;
				}
				front_laser.laser_pose.x = odometry.x;
				front_laser.laser_pose.y = odometry.y;
				front_laser.laser_pose.theta = 
				  odometry.theta;

 				front_laser.robot_pose.x = odometry.x;
				front_laser.robot_pose.y = odometry.y;
				front_laser.robot_pose.theta = 
				  odometry.theta;

				if(first) {
					first_timestamp = timestamp;
					first = 0;
				}
				front_laser_timestamp = timestamp - first_timestamp;

				if(current_front_laser > 0 && 
					 front_laser_timestamp < last_front_laser_timestamp)
					front_laser_timestamp = last_front_laser_timestamp;

//				type_id = ROBOT_FRONTLASER_ID;

				front_laser.timestamp = front_laser_timestamp;
				
				carmen_logwrite_write_robot_laser(&front_laser, 1, outfile, front_laser_timestamp);
	  
				current_front_laser++;
      }
      else if(strncmp(line2, "#ROBOT", 6) == 0) {
				last_odometry_timestamp = odometry_timestamp;
	
				sscanf(line, "@SENS %d-%d-%d %d:%d:%f\n", &m, &d, &y, &h, &min, &s);
				timestamp = h * 3600.0 + min * 60.0 + s;
				fscanf(fp, " %lf %lf %lf", &odometry.x, &odometry.y, 
							 &odometry.theta);
				odometry.x /= 100.0;
				odometry.y /= 100.0;
				odometry.theta = carmen_normalize_theta(carmen_degrees_to_radians(odometry.theta));
	
				if(first) {
					first_timestamp = timestamp;
					first = 0;
				}
				odometry_timestamp = timestamp - first_timestamp;

				if(current_odometry > 0 &&
					 odometry_timestamp < last_odometry_timestamp)
					odometry_timestamp = last_odometry_timestamp;

//				type_id = ODOM_ID;
 				carmen_logwrite_write_odometry(&odometry, outfile, odometry_timestamp); 

				current_odometry++;
      }
      fgets(line2, LINE_SIZE, fp);
    }
  }

  fclose(fp);
  carmen_fclose(outfile);
  return 0;
}
