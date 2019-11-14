#include <carmen/carmen.h>

int camera;
double bumb_latest_timestamp;
carmen_point_t globalpos;
carmen_pose_3D_t pose;
carmen_behavior_selector_road_profile_message last_rddf_poses;
int rddf_received = 0;
int bumblebee_received = 0;


double
euclidean_distance (double x1, double x2, double y1, double y2)
{
	return ( sqrt(pow(x2-x1,2) + pow(y2-y1,2)) );
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
rddf_handler(carmen_behavior_selector_road_profile_message *message)
{
	last_rddf_poses = *message;
	rddf_received = 1;
}

void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumb_latest_timestamp = image_msg->timestamp;
	bumblebee_received = 1;
}

void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	if (bumblebee_received && rddf_received)
	{
		globalpos.theta = globalpos_message->globalpos.theta;
		globalpos.x = globalpos_message->globalpos.x;
		globalpos.y = globalpos_message->globalpos.y;
		double current_timestamp = globalpos_message->timestamp;

		int index_aux=0;
		double distance, min_distance = DBL_MAX;
		for (int i = 0; i < last_rddf_poses.number_of_poses; i++)
		{
			distance = euclidean_distance(globalpos.x, last_rddf_poses.poses[i].x, globalpos.y, last_rddf_poses.poses[i].y);
			if (distance < min_distance)
			{
				min_distance = distance;
				index_aux = i;
			}
		}
		printf("%d\n", index_aux);

		printf("pose: %.2f %.2f %.2f %.2f\n", globalpos.x, globalpos.y, globalpos.theta, globalpos_message->timestamp);
		printf("rddf: %.2f %.2f %.2f\n", last_rddf_poses.poses[index_aux].x, last_rddf_poses.poses[index_aux].y, last_rddf_poses.poses[index_aux].theta);
		printf("image: %f\n\n", bumb_latest_timestamp);
	}
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        printf("Subscribe_example: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Subscribes                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
subscribe_messages()
{
	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, (char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT,
					NULL, sizeof (carmen_behavior_selector_road_profile_message), (carmen_handler_t) rddf_handler, CARMEN_SUBSCRIBE_LATEST);
}

void
read_parameters(int argc, char **argv)
{
	camera = atoi(argv[1]);
}


int
main(int argc , char **argv)
{
	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	read_parameters(argc, argv);

	printf("Aguardando mensagem\n");

	subscribe_messages();

	carmen_ipc_dispatch();

	return 0;
}
