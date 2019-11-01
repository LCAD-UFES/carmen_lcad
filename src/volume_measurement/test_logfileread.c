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

int main(int argc, char **argv)
{
  int i, percent, last_percent = 0;
  char line[100000];
  int odometry_count = 0;

  carmen_base_odometry_message odometry;
  carmen_erase_structure(&odometry, sizeof(carmen_base_odometry_message) );

  /* initialize connection to IPC network */
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);  

  /* open the logfile */
  logfile = carmen_fopen(argv[1], "r");
  if(logfile == NULL)
    carmen_die("Error: could not open file %s for reading.\n", argv[1]);

  /* index the logfile */
  logfile_index = carmen_logfile_index_messages(logfile);

  for(i = 0; i < logfile_index->num_messages; i++) {
    /* print out percentage read */
    percent = (int)floor(i / (double)logfile_index->num_messages * 100.0);
    if(percent > last_percent) {
      fprintf(stderr, "\rReading logfile (%d%%)   ", percent);
      last_percent = percent;
    }
    
    /* read i-th line */
    carmen_logfile_read_line(logfile_index, logfile, i, 100000, line);

    /* read odometry message */
    if(strncmp(line, "ODOM ", 5) == 0) {
      carmen_string_to_base_odometry_message(carmen_next_word(line), 
					     &odometry);
      odometry_count++;
    }
  }
  fprintf(stderr, "\rReading logfile (100%%)   \n");

  fprintf(stderr, "Read %d odometry messages.\n", odometry_count);
  return 0;
}
