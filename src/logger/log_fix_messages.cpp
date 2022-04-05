 /*********************************************************
 * Code to fix ROBOTVELOCITY_ACK message inside a log.
 * Vinicius
 ********************************************************/

#include <fcntl.h>
#include <carmen/carmen.h>
#include "readlog.h"
#include <carmen/playback_interface.h>

#define        MAX_LINE_LENGTH           (5*4000000)

carmen_FILE *logfile = NULL;
carmen_FILE *logfile_to_fix = NULL;
char *log_filename = NULL;
carmen_logfile_index_p logfile_index = NULL;
carmen_robot_ackerman_velocity_message *velocity_ackerman_msg;

int main(int argc, char **argv)
{
  int v_changed = 0;
  int phi_changed = 0;
  int i, percent, last_percent = 0;
  char line[100000];
  int messages_count = 0;
  int messages_changed = 0;
  double log_playback_time;
  char *host_carmen;
  double last_v = 0;
  carmen_FILE *outfile = NULL;

  host_carmen = (char*) malloc (64 * sizeof(char));
  memset(&velocity_ackerman_msg, 0, sizeof(velocity_ackerman_msg));

  if (argc < 7 || (argc > 1 && strcmp(argv[1], "-h") == 0))
  {
	  fprintf(stderr, "\n This program is specific to correct the message ROBOTVELOCITY_ACK. There are 2 options \n"
			  "(i) to correct the velocity (ex: negative velocities between positive)\n"
			  "(ii) to correct the phi when the odometry is bad. This program will change all phi to 0.0\n");
	  fprintf(stderr, " This program will create a new file, however SAVE copy of your log.txt \n "
			  "then after generate the new file, rename it back to the original file, the _images and _lidar folders use the original log name\n");
	  fprintf(stderr, "\nUsage:   %s  -v <0(off)/1(on)> -phi <0(off)/1(on)> </path/input_log_camen.txt> <output_log_file.txt> \n", argv[0]);

	  exit(-1);
  }

  int fix_v = atoi(argv[2]);
  int fix_phi = atoi(argv[4]);
  log_filename = argv[5];
  char* log_output_filename = argv[6];

  outfile = carmen_fopen(log_output_filename, "r");

  if (outfile != NULL)
  {
	  fprintf(stderr, "Overwrite %s (Y/N)? ", log_output_filename);
	  char key;
	  scanf("%c", &key);
	  if (toupper(key) != 'Y')
		  carmen_die("Log file %s is not supposed to be overwritten!\n", log_output_filename);
	  carmen_fclose(outfile);
  }
  outfile = carmen_fopen(log_output_filename, "w");

  if (outfile == NULL)
	  carmen_die("Error: Could not open file %s for writing.\n", log_output_filename);

//  carmen_logwrite_write_header(outfile);

  /* initialize connection to IPC network */
//  carmen_ipc_initialize(argc, argv);
//  carmen_param_check_version(argv[0]);

  /* open the logfile */
  logfile_to_fix = carmen_fopen(log_filename, "r");
  if(logfile_to_fix == NULL)
    carmen_die("Error: could not open file %s for reading.\n", argv[1]);

  /* index the logfile */
  logfile_index = carmen_logfile_index_messages(logfile_to_fix);

  for(i = 0; i < logfile_index->num_messages; i++) {
    /* print out percentage read */
    percent = (int)floor(i / (double)logfile_index->num_messages * 100.0);
    if(percent > last_percent) {
      fprintf(stderr, "\rReading logfile (%d%%)   ", percent);
      last_percent = percent;
    }

    /* read i-th line */
    carmen_logfile_read_line(logfile_index, logfile_to_fix, i, MAX_LINE_LENGTH, line);

    char *current_position;

    if (strncmp(line, "ROBOTVELOCITY_ACK ", 18) == 0)
    {
    	current_position = carmen_next_word(line);
    	double v = CLF_READ_DOUBLE(&current_position);
    	double phi = CLF_READ_DOUBLE(&current_position);
    	double timestamp = CLF_READ_DOUBLE(&current_position);
    	copy_host_string(&host_carmen, &current_position);
    	log_playback_time = atof(current_position);

    	//DO SOMETHING
    	if(fix_v)
    	{
    		if(carmen_sign(v) != carmen_sign(last_v) &&
    				(fabs(last_v) > 0.005))
    		{
    			v = (-1.0) * v;
    			v_changed = 1;
    		}
    		last_v = v;
    	}

    	if(fix_phi)
    	{
    		phi = 0.0;
    		phi_changed = 1;
    	}

    	if (v_changed || phi_changed)
    	{
    		messages_changed++;
    		v_changed = 0;
    		phi_changed = 0;
    	}

    	carmen_fprintf(outfile, "ROBOTVELOCITY_ACK %f %f %f %s %f\n", v,
    			phi, timestamp, host_carmen, log_playback_time);
    	messages_count++;
    }
    else
    {
    	carmen_fprintf(outfile, line);
    	//write file

    }
  }
  carmen_fclose(outfile);
  carmen_fclose(logfile_to_fix);

  fprintf(stderr, "\rReading logfile (100%%)   \n");

  fprintf(stderr, "Read %d ROBOTVELOCITY_ACK messages.\n", messages_count);
  fprintf(stderr, "Changed %d ROBOTVELOCITY_ACK messages.\n", messages_changed);
  return 0;
}
