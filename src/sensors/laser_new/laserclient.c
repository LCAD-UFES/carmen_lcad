#include <carmen/carmen.h>
#include "laser_interface.h"
#include <stdio.h>

static void 
laser_handler(carmen_laser_laser_message *laser)
{
 fprintf(stderr, "n_ranges %d, n_remissions %d, fov %lf, maxrange %lf, timestamp %.3lf, host %s\n",
	laser->num_readings, laser->num_remissions, laser->config.fov, laser->config.maximum_range, laser->timestamp, laser->host);
}


int 
main(int argc, char **argv)
{  
    int i=atoi(argv[1]);
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);
    fprintf(stderr, "dumping laser %d\n", i);
    static carmen_laser_laser_message laser;
  carmen_laser_subscribe_laser_message(i,&laser, (carmen_handler_t)
					      laser_handler,
					      CARMEN_SUBSCRIBE_LATEST);
  
  while(1) IPC_listen(10);
  return 0;
}


