 /*********************************************************
	---   Skeleton Module Application ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/simple_2d_mapper_interface.h>

#include "simple_2d_mapper_main.h"

Mapper2D* mapper;
static scan_data_t velodyne_scan;

carmen_fused_odometry_message* fused_odometry_message_list_g = NULL;
int fused_odometry_message_size_g = 0;
int count_fused_odometry_message_g = 0;

void initialize_fused_odometry_message_list(int size_of_list)
{
	fused_odometry_message_size_g = size_of_list;

	if(fused_odometry_message_list_g == NULL)
		fused_odometry_message_list_g = (carmen_fused_odometry_message*) malloc (fused_odometry_message_size_g * sizeof(carmen_fused_odometry_message));

}

void add_message_to_fused_odometry_message_list(carmen_fused_odometry_message* message)
{
	fused_odometry_message_list_g[count_fused_odometry_message_g].pose = message->pose;
	fused_odometry_message_list_g[count_fused_odometry_message_g].velocity = message->velocity;
	fused_odometry_message_list_g[count_fused_odometry_message_g].angular_velocity = message->angular_velocity;
	fused_odometry_message_list_g[count_fused_odometry_message_g].timestamp = message->timestamp;

	if(count_fused_odometry_message_g == fused_odometry_message_size_g - 1)
		count_fused_odometry_message_g = 0;

	count_fused_odometry_message_g++;
}

carmen_pose_3D_t get_fused_odometry_pose_interpolated_with_velodyne_scan(carmen_velodyne_partial_scan_message* message)
{
	int fused_odometry_index = 0;
	double curr_timestamp_diff;
	double min_timestamp_diff = 99999999.0;

	for(int i = 0; i < fused_odometry_message_size_g; i++)
	{
		curr_timestamp_diff = fabs(fused_odometry_message_list_g[i].timestamp - message->timestamp);

		if(curr_timestamp_diff < min_timestamp_diff)
		{
			min_timestamp_diff = curr_timestamp_diff;
			fused_odometry_index = i;
		}
	}

	return fused_odometry_message_list_g[fused_odometry_index].pose;
}

void initialize_velodyne_360_shots()
{
	for(int i=0; i < 360; i++)
	{
		velodyne_scan.scan[i].visited = false;
		velodyne_scan.scan[i].shots.clear();
	}
}

bool visited_all_velodyne_360_shots()
{
	for(int i=0; i < 360; i++)
	{
		if(velodyne_scan.scan[i].visited == false)
			return false;
	}

	return true;
}

/*********************************************************
		   --- Publishers ---
**********************************************************/


/*********************************************************
		   --- Handlers ---
**********************************************************/
void carmen_fused_odometry_handler(carmen_fused_odometry_message* message)
{
	add_message_to_fused_odometry_message_list(message);
}

long int map_counter = 0;

void
carmen_velodyne_handler(carmen_velodyne_partial_scan_message* message)
{
	int velodyne_360_shot_position = 0;
	char map_name[256];

	for(int i = 0; i < message->number_of_32_laser_shots; i++)
	{
		velodyne_360_shot_position = (int) floor(message->partial_scan[i].angle);
		velodyne_scan.scan[velodyne_360_shot_position].shots.push_back(message->partial_scan[i]);

		if(velodyne_scan.scan[velodyne_360_shot_position].visited == false)
			velodyne_scan.scan[velodyne_360_shot_position].visited = true;

		if(visited_all_velodyne_360_shots())
		{
				velodyne_scan.pose = get_fused_odometry_pose_interpolated_with_velodyne_scan(message);
				mapper->InsertScanData(velodyne_scan);
				sprintf(map_name, "map_%ld.pgm", map_counter);
				map_counter++;
				mapper->SaveImageMap(map_name, mapper->map_1d, (int)(1000.0/0.5), (int)(3000.0/0.5));
				initialize_velodyne_360_shots();
		}
	}
}

void static
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("simple_2d_mapper: disconnected.\n");

    exit(0);
  }
}
  
int 
main(int argc, char **argv) 
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  initialize_fused_odometry_message_list(100);
  mapper = new Mapper2D(1000, 3000, 0.1f);

  /* Define messages that your module publishes */
  //carmen_skeleton_module_filter_define_messages();

  /* Subscribe to sensor messages */
  carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) carmen_velodyne_handler, CARMEN_SUBSCRIBE_LATEST);
  carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) carmen_fused_odometry_handler, CARMEN_SUBSCRIBE_LATEST);

  /* Loop forever waiting for messages */
  carmen_ipc_dispatch();

  return (0);
}
