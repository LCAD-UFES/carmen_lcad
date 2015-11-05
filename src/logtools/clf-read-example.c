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
#include <carmen/carmen_stdio.h>

/* this programm shows an example how to read log files
   with the readlog library provided by logger           */

int main(int argc, char* argv[]) {

  if (argc != 2) 
    carmen_die("Syntax: %s <carmen-logfile>\n", argv[0]);


  /*   file handler */
  carmen_FILE *logfile = NULL;

  /*   log file index structure */
  carmen_logfile_index_p logfile_index = NULL;

  /*  open logfile for reading */
  logfile = carmen_fopen(argv[1], "r");
  if(logfile == NULL)
    carmen_die("Error: could not open file %s for reading.\n", argv[1]);

  /* create index structure */
  logfile_index = carmen_logfile_index_messages(logfile);
  
  /*   buffer for reading the lines */
  const int max_line_length=9999;
  char line[max_line_length+1];
  
  /*   used to read the messages. erase stucture before! */
  carmen_robot_laser_message l;
  carmen_erase_structure(&l, sizeof(carmen_robot_laser_message) );
  carmen_laser_laser_message rawl;
  carmen_erase_structure(&l, sizeof(carmen_laser_laser_message) );
  carmen_base_odometry_message o;
  carmen_erase_structure(&o, sizeof(carmen_base_odometry_message) );

  /*   iterate over all entries in the logfile */
  while (!carmen_logfile_eof(logfile_index)) {
    
    /*     read the line from the file */
    carmen_logfile_read_next_line(logfile_index, 
				  logfile,
				  max_line_length, 
				  line);
    
    /*     what kind of message it this? */
    if (strncmp(line, "ROBOTLASER1 ", 12) == 0) {
      /*  convert the string using the corresponding read function */
      carmen_string_to_robot_laser_message(line, &l);
      fprintf(stderr, "F");
    }
    else if (strncmp(line, "ROBOTLASER2 ", 12) == 0) {
      /*  convert the string using the corresponding read function */
      carmen_string_to_robot_laser_message(line, &l);
      fprintf(stderr, "R");
    }
    else if (strncmp(line, "FLASER ", 7) == 0) {
      /*  convert the string using the corresponding read function */
      carmen_string_to_robot_laser_message_orig(line, &l);
      fprintf(stderr, "f");
    }
    else if (strncmp(line, "RLASER", 7) == 0) {
      /*  convert the string using the corresponding read function */
      carmen_string_to_robot_laser_message_orig(line, &l);
      fprintf(stderr, "r");
    }
    else if (strncmp(line, "RAWLASER", 8) == 0) {
      /*  convert the string using the corresponding read function */
      carmen_string_to_laser_laser_message(line, &rawl);
      fprintf(stderr, "L");
    }
    else if (strncmp(line, "ODOM ", 5) == 0) {
      /*  convert the string using the corresponding read function */
      carmen_string_to_base_odometry_message(line, &o);
      fprintf(stderr, "o");
    }
  }

  /* close the file */
  carmen_fclose(logfile);

  fprintf(stderr, "\ndone!\n");

  /* free index structure */
  carmen_logfile_free_index(&logfile_index);
  return 0;    
}
