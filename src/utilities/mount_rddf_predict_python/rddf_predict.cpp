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
FILE *file_log2;
FILE *file_log3;
int first_it = 0;
int first_it2 = 0;
int first_it3 = 0;
char txt_name[150];
char txt_name2[150];
char txt_name3[150];
char save_buffer[500];
char save_buffer2[500];
char save_buffer3[2000];


double y_check = -1000.0;

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
	snprintf(save_buffer, sizeof(save_buffer),"%f %f %f %f %f %f", dy, dtheta, k1, k2, k3, bumb_latest_timestamp);
	//snprintf(save_buffer, sizeof(save_buffer),"%f %f %f %f#%f %f %f#%f", robot_x, robot_y, robot_theta, robot_timestamp, rddf_x, rddf_y, rddf_theta, bumb_image);
	fprintf(file_log,"%s\n", save_buffer);

}

void
save_waypoint_to_txt(int index_aux, carmen_behavior_selector_road_profile_message *rddf_poses, carmen_ackerman_traj_point_t *rddf_poses_global, double bumb_latest_timestamp, carmen_point_t global_iara_pose)
{
	if(!first_it3)
	{
		memset(save_buffer3,'\0', 2000*sizeof(char));
		memset(txt_name3,'\0',150*sizeof(char));
		first_it3 = 1;
		struct stat sb;
		if(!(stat("/dados/rddf_predict", &sb) == 0))
			mkdir("/dados/rddf_predict",0777);
		time_t t = time(NULL);
		struct tm tm = *localtime(&t);
		snprintf(txt_name3, sizeof(txt_name3), "/dados/rddf_predict/rddf_ground_truth_%d-%d-%d_%d:%d:%d",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		file_log3 = fopen(txt_name3, "a");

		if(NULL == file_log3)
		{
			printf("Erro ao abrir o arquivo log_rddf_predict.txt no método save_waypoint_to_txt (rddf_predict.c)\n");
			exit(1);
		}
	}
	snprintf(save_buffer3, sizeof(save_buffer3),"%f %f %f %f ", bumb_latest_timestamp, global_iara_pose.x, global_iara_pose.y, global_iara_pose.theta);
	fprintf(file_log3,"%s", save_buffer3);


	for (int i = index_aux; rddf_poses->poses[i].x <= 30.0; i++) {
		if (rddf_poses->poses[i].x > 0.0) {
			snprintf(save_buffer3, sizeof(save_buffer3),"%f %f ", rddf_poses_global[i].x, rddf_poses_global[i].y);
			fprintf(file_log3,"%s", save_buffer3);
		}

		}

//	snprintf(save_buffer3, sizeof(save_buffer3),"%f %f %f %f %f %f", dy, dtheta, k1, k2, k3, bumb_latest_timestamp);
	//snprintf(save_buffer3, sizeof(save_buffer3),"%f %f %f %f#%f %f %f#%f", robot_x, robot_y, robot_theta, robot_timestamp, rddf_x, rddf_y, rddf_theta, bumb_image);
	fprintf(file_log3,"\n");

	//last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, image_msg->timestamp


}


void
save_for_plotting(double k1, double k2, double k3)
{
	if(!first_it2)
	{
		memset(save_buffer2,'\0',500*sizeof(char));
		memset(txt_name2,'\0',150*sizeof(char));
		first_it2 = 1;
		struct stat sb;
		if(!(stat("/dados/rddf_plots", &sb) == 0))
			mkdir("/dados/rddf_plots",0777);
		time_t t = time(NULL);
		struct tm tm = *localtime(&t);
		snprintf(txt_name2, sizeof(txt_name2), "/dados/rddf_plots/listen_%d-%d-%d_%d:%d:%d",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		file_log2 = fopen(txt_name2, "a");

		if(NULL == file_log2)
		{
			printf("Erro ao abrir o arquivo log_rddf_predict.txt no método save_for_plotting (rddf_predict.c)\n");
			exit(1);
		}
	}

	// gera y's do spline
	gsl_interp_accel *acc;
	gsl_spline *phi_spline;
	double knots_x[4] = {0.0,  30/ 3.0, 2 * 30 / 3.0, 30.0};
	double knots_y[4] = {0.0, k1, k2, k3};
	acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	phi_spline = gsl_spline_alloc(type, 4);
	gsl_spline_init(phi_spline, knots_x, knots_y, 4);

	if ((last_rddf_poses.poses[10].y != y_check) || (y_check == -1000.0))
	{
		y_check = last_rddf_poses.poses[10].y;
		for(int i = 0; (i<last_rddf_poses.number_of_poses) && (last_rddf_poses.poses[i].x < 30.0); i++)
		{
			if (last_rddf_poses.poses[i].x >= 0.0)
			{
				double spline_y = gsl_spline_eval(phi_spline, last_rddf_poses.poses[i].x, acc);
				snprintf(save_buffer2, sizeof(save_buffer2),"%f %f %f", last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, spline_y);
				//snprintf(save_buffer, sizeof(save_buffer),"%f %f %f %f#%f %f %f#%f", robot_x, robot_y, robot_theta, robot_timestamp, rddf_x, rddf_y, rddf_theta, bumb_image);
				fprintf(file_log2,"%s\n", save_buffer2);
			}
		}
	}

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
				//printf("coord %f %f\n", last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y);
				distance = euclidean_distance(globalpos.x, last_rddf_poses.poses[i].x, globalpos.y, last_rddf_poses.poses[i].y);
				if (distance < min_distance)
				{
					min_distance = distance;
					index_aux = i;
				}
			}
			//printf("%f, %f ### %f %f", last_rddf_poses.poses[index_aux].x, last_rddf_poses.poses[index_aux].y, last_rddf_poses.poses[index_aux+1].x, last_rddf_poses.poses[index_aux+1].y);
			/*
			printf("pose: %.2f %.2f %.2f %.2f\n", globalpos.x, globalpos.y, globalpos.theta, globalpos_message->timestamp);
			printf("rddf: %.2f %.2f %.2f\n", last_rddf_poses.poses[index_aux].x, last_rddf_poses.poses[index_aux].y, last_rddf_poses.poses[index_aux].theta);
			printf("image: %f\n\n", bumb_latest_timestamp);
			 */
			double dtheta = globalpos.theta - last_rddf_poses.poses[index_aux].theta;

//			printf("rddf: %.2f %.2f %.2f\n", last_rddf_poses.poses[index_aux].x, last_rddf_poses.poses[index_aux].y, last_rddf_poses.poses[index_aux].theta);

			SE2 rddf_pose(last_rddf_poses.poses[index_aux].x, last_rddf_poses.poses[index_aux].y, last_rddf_poses.poses[index_aux].theta);

			carmen_ackerman_traj_point_t global_poses[last_rddf_poses.number_of_poses];

			for (int i = 0; i < last_rddf_poses.number_of_poses; i++)
			{
				global_poses[i].x = last_rddf_poses.poses[i].x;
				global_poses[i].y = last_rddf_poses.poses[i].y;
				SE2 lane_in_world_reference(last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, last_rddf_poses.poses[i].theta);
				SE2 lane_in_rddf_reference = rddf_pose.inverse() * lane_in_world_reference;
				last_rddf_poses.poses[i].x = lane_in_rddf_reference[0];
				last_rddf_poses.poses[i].y = lane_in_rddf_reference[1];
				last_rddf_poses.poses[i].theta = lane_in_rddf_reference[2];
			}

			save_waypoint_to_txt(index_aux ,&last_rddf_poses, global_poses, image_msg->timestamp, globalpos );


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
				printf("%f // %f // %f // %f\n", dy, globalpos.x, globalpos.y, globalpos.theta);

			}
			printf("%f\n",image_msg->timestamp);
			save_to_txt(dy, dtheta, spc.k1, spc.k2, spc.k3, image_msg->timestamp);
			save_for_plotting(spc.k1, spc.k2, spc.k3);

		}
	rddf_received = 0;
	localize_received = 0;

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
    	fclose(file_log2);
    	fclose(file_log3);

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
