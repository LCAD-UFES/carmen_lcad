#include <carmen/carmen.h>
#include "g2o/types/slam2d/se2.h"
#include "rddf_predict_optimizer.h"

using namespace g2o;

int camera;
carmen_localize_ackerman_globalpos_message ackerman_message;
carmen_point_t globalpos;
carmen_pose_3D_t pose;
carmen_behavior_selector_road_profile_message last_rddf_poses;
int rddf_received = 0;
int localize_received = 0;

FILE *file_log;
int first_it = 0;
char txt_name[150];
char save_buffer[500];

double
euclidean_distance (double x1, double x2, double y1, double y2)
{
	return ( sqrt(pow(x2-x1,2) + pow(y2-y1,2)) );
}
//save_to_txt(double robot_x, double robot_y, double robot_theta, double robot_timestamp, double rddf_x, double rddf_y, double rddf_theta, double bumb_image)
void
save_to_txt(double dy, double dtheta, double k1, double k2, double k3, double bumb_latest_timestamp)
{
	if(!first_it)
	{
		memset(save_buffer,'\0',500*sizeof(char));
		memset(txt_name,'\0',150*sizeof(char));
		first_it = 1;
		struct stat sb;
		if(!(stat("/dados/rddf_predict", &sb) == 0))
			mkdir("/dados/rddf_predict",0777);
		time_t t = time(NULL);
		struct tm tm = *localtime(&t);
		snprintf(txt_name, sizeof(txt_name), "/dados/rddf_predict/listen_%d-%d-%d_%d:%d:%d",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		file_log = fopen(txt_name, "a");

		if(NULL == file_log)
		{
			printf("Erro ao abrir o arquivo log_rddf_predict.txt no método save_to_txt (rddf_predict.c)\n");
			exit(1);
		}
	}
	snprintf(save_buffer, sizeof(save_buffer),"%f %f#%f %f %f#%f", dy, dtheta, k1, k2, k3, bumb_latest_timestamp);
	//snprintf(save_buffer, sizeof(save_buffer),"%f %f %f %f#%f %f %f#%f", robot_x, robot_y, robot_theta, robot_timestamp, rddf_x, rddf_y, rddf_theta, bumb_image);
	fprintf(file_log,"%s\n", save_buffer);

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
	if (localize_received && rddf_received)
		{
			globalpos.theta = ackerman_message.globalpos.theta;
			globalpos.x = ackerman_message.globalpos.x;
			globalpos.y = ackerman_message.globalpos.y;

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
			/*
			printf("pose: %.2f %.2f %.2f %.2f\n", globalpos.x, globalpos.y, globalpos.theta, globalpos_message->timestamp);
			printf("rddf: %.2f %.2f %.2f\n", last_rddf_poses.poses[index_aux].x, last_rddf_poses.poses[index_aux].y, last_rddf_poses.poses[index_aux].theta);
			printf("image: %f\n\n", bumb_latest_timestamp);
			 */
			double dtheta = globalpos.theta - last_rddf_poses.poses[index_aux].theta;

			SE2 rddf_pose(last_rddf_poses.poses[index_aux].x, last_rddf_poses.poses[index_aux].y, last_rddf_poses.poses[index_aux].theta);

			for (int i = 0; i < last_rddf_poses.number_of_poses; i++)
			{
				SE2 lane_in_world_reference(last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, last_rddf_poses.poses[i].theta);
				SE2 lane_in_rddf_reference = rddf_pose.inverse() * lane_in_world_reference;
				last_rddf_poses.poses[i].x = lane_in_rddf_reference[0];
				last_rddf_poses.poses[i].y = lane_in_rddf_reference[1];
				last_rddf_poses.poses[i].theta = lane_in_rddf_reference[2];
			}

			SE2 car_in_world_reference(globalpos.x, globalpos.y, globalpos.theta);
			SE2 car_in_rddf_reference = rddf_pose.inverse() * car_in_world_reference;

			double dy = car_in_rddf_reference[1];

			/*
			for (int i = 0; i < last_rddf_poses.number_of_poses; i++)
			{
				printf("%f %f %f\n", last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, last_rddf_poses.poses[i].theta);
			}
			*/

			SplineControlParams spc = optimize_spline_knots(&last_rddf_poses);

			/*
			printf("dy = %f\ndtheta = %f\n\nposes:\n", dy, dtheta);
			printf("k1 = %f, k2 = %f, k3 = %f\n", spc.k1, spc.k2, spc.k3);
			*/
			//save_to_txt(globalpos.x, globalpos.y, globalpos.theta, globalpos_message->timestamp, last_rddf_poses.poses[index_aux].x,
			//		last_rddf_poses.poses[index_aux].y, last_rddf_poses.poses[index_aux].theta, bumb_latest_timestamp);
			if(dy>10)
			{
				printf("%f\n",dy);

			}
			save_to_txt(dy, dtheta, spc.k1, spc.k2, spc.k3, image_msg->timestamp);

		}

}

void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	localize_received = 1;
	ackerman_message = *globalpos_message;

}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
    	fclose(file_log);
        carmen_ipc_disconnect();
        printf("Rddf_predict: Disconnected.\n");
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
read_parameters(char **argv)
{
	camera = atoi(argv[1]);
}


int
main(int argc , char **argv)
{
	if(argc!=2)
	{
		printf("É necessário passar o ID da câmera como parâmetro.");
		exit(1);
	}

	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	read_parameters(argv);

	printf("Aguardando mensagem\n");

	subscribe_messages();

	carmen_ipc_dispatch();

	return 0;
}
