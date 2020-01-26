#include <carmen/carmen.h>
#include "g2o/types/slam2d/se2.h"
#include "rddf_predict_optimizer.h"
#include <vector>
#include <algorithm>
#include <fstream>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_index.h>

using namespace g2o;

int camera;
carmen_localize_ackerman_globalpos_message ackerman_message;
carmen_point_t globalpos;
carmen_pose_3D_t pose;
carmen_behavior_selector_road_profile_message last_rddf_poses;
int rddf_received = 0;
int localize_received = 0;
carmen_ackerman_traj_point_t last_pose = {.x = 0.0, .y = 0.0, .theta = 0.0, .v = 9.0, .phi=0.2};


FILE *file_log;
FILE *file_log2;
int first_it = 0;
int first_it2 = 0;
char txt_name[150];
char save_buffer[500];
char txt_name2[150];
char save_buffer2[500];

double y_check = -1000.0;

struct Prediction
{
    double dy;
    double dtheta; //temporariamente não usado
    double k1;
    double k2;
    double k3;
    double timestamp;
};

struct mystruct_comparer
{
    bool operator()(Prediction const& ms, double const i) const
    {
        return ms.timestamp < i;
    }
};

std::vector<Prediction> preds;

std::istream& operator>>(std::istream& is, Prediction& s)
{
//    return is >> s.dy >> s.dtheta >> s.k1 >> s.k2 >> s.k3 >> s.timestamp;
	return is >> s.dy >> s.k1 >> s.k2 >> s.k3 >> s.timestamp;
}

double
euclidean_distance (double x1, double x2, double y1, double y2)
{
	return ( sqrt(pow(x2-x1,2) + pow(y2-y1,2)) );
}

bool compareByLength(const Prediction &a, const Prediction &b)
{
    return a.timestamp < b.timestamp;
}

void
plot_state(std::vector<carmen_ackerman_traj_point_t> &spline_vec, carmen_point_t iara_pose)
{
//	plot data Table - Last TCP - Optmizer tcp - Lane
	//Plot Optmizer step tcp and lane?

	#define DELTA_T (1.0 / 40.0)

//	#define PAST_SIZE 300
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;


	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xrange [0:70]\n");
		fprintf(gnuplot_pipeMP, "set yrange [-10:10]\n");
//		fprintf(gnuplot_pipe, "set y2range [-0.55:0.55]\n");
		fprintf(gnuplot_pipeMP, "set xlabel 'senconds'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'effort'\n");
//		fprintf(gnuplot_pipe, "set y2label 'phi (radians)'\n");
//		fprintf(gnuplot_pipe, "set ytics nomirror\n");
//		fprintf(gnuplot_pipe, "set y2tics\n");
		fprintf(gnuplot_pipeMP, "set tics out\n");
	}

	FILE *gnuplot_spline = fopen("gnuplot_spline_poses.txt", "w");
	FILE *gnuplot_globalpos = fopen("gnuplot_data_iara_pose.txt", "w");

	for (unsigned int i = 0; i < spline_vec.size(); i++)
		fprintf(gnuplot_spline, "%lf %lf %lf %lf %lf\n", spline_vec.at(i).x, spline_vec.at(i).y, 1.0 * cos(spline_vec.at(i).theta), 1.0 * sin(spline_vec.at(i).theta), spline_vec.at(i).theta);
	fprintf(gnuplot_globalpos, "%lf %lf\n", iara_pose.x, iara_pose.y);

	fclose(gnuplot_spline);
	fclose(gnuplot_globalpos);

//	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, %lf to %lf, %lf nohead\n",0, -60.0, 0, 60.0);

	fprintf(gnuplot_pipeMP, "plot "
			"'./gnuplot_spline_poses.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'spline',"
			"'./gnuplot_data_iara_pose.txt' using 1:2 with p title 'iara_pose' axes x1y1\n");

	fflush(gnuplot_pipeMP);
}

//vector<carmen_position_t>
//get_rddf_points_in_image_full (tf::StampedTransform world_to_camera_pose, int img_width, int img_height)
//{
//	carmen_position_t p;
//	vector<carmen_position_t> rddf_points_in_image;
//	double distance, last_distance;
//	for(int i = 0; i < last_rddf_poses.number_of_poses; i++)
//	{
//		p = convert_rddf_pose_to_point_in_image (last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, 0.0, world_to_camera_pose, camera_parameters, img_width, img_height);
//		rddf_points_in_image.push_back(p);
//	}
//	return (rddf_points_in_image);
//}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	if (localize_received)
	{
		globalpos.theta = ackerman_message.globalpos.theta;
		globalpos.x = ackerman_message.globalpos.x;
		globalpos.y = ackerman_message.globalpos.y;
		//printf("%f %f\n", globalpos.x, globalpos.y);

		double img_timestamp = image_msg->timestamp;

//			Prediction *found = std::lower_bound(preds.front(),
//											   preds.back(),
//											   img_timestamp,
//											   mystruct_comparer());
		int index = -1;
		for(int i = 0; i<preds.size(); i++)
		{
			if (preds[i].timestamp == img_timestamp)
			{
				index = i;
				break;
			}
			else if (preds[i].timestamp > img_timestamp)
				break;
		}

		if(index != -1)
		{
			printf("%f\n", preds[index].timestamp);

			gsl_interp_accel *acc;
			gsl_spline *phi_spline;
			double knots_x[4] = {0.0,  30/ 3.0, 2 * 30 / 3.0, 30.0};
			double knots_y[4] = {0.0, preds[index].k1, preds[index].k2, preds[index].k3};
			acc = gsl_interp_accel_alloc();
			const gsl_interp_type *type = gsl_interp_cspline;
			phi_spline = gsl_spline_alloc(type, 4);
			gsl_spline_init(phi_spline, knots_x, knots_y, 4);


			double half_points = 0.0;
			double acresc_points = 0.01;
			double store_y[int(30/acresc_points)+1];
			double store_x[int(30/acresc_points)+1];
			double store_thetas[int(30/acresc_points)+1];
			double points_dx = 0.1;
			int indice_points = 0;
			//for(int i = 0; i < 30*2 ; i++)
			while( half_points <= 30.0 )
			{
				if(half_points >= 29.9)
				{
					double spline_y = gsl_spline_eval(phi_spline, half_points, acc);
					if (euclidean_distance(half_points, store_x[indice_points - 1], spline_y, store_y[indice_points - 1]) >= 0.5)
					{
						double spline_y2 = gsl_spline_eval(phi_spline, half_points - points_dx, acc);
						store_y[indice_points] = spline_y;
						store_x[indice_points] = half_points;
						store_thetas[indice_points] = carmen_normalize_theta(atan2(spline_y - spline_y2, points_dx));
						indice_points++;
					}
				}
				else if(half_points == 0.0)
				{
					double spline_y = gsl_spline_eval(phi_spline, half_points, acc);
					double spline_y2 = gsl_spline_eval(phi_spline, half_points + points_dx, acc);
					store_y[indice_points] = spline_y;
					store_x[indice_points] = half_points;
					store_thetas[indice_points] = carmen_normalize_theta(atan2(spline_y2 - spline_y, points_dx));
					indice_points++;
				}
				else
				{
					double spline_y = gsl_spline_eval(phi_spline, half_points, acc);
					if (euclidean_distance(half_points, store_x[indice_points - 1], spline_y, store_y[indice_points - 1]) >= 0.5)
					{
						double spline_y2 = gsl_spline_eval(phi_spline, half_points + points_dx, acc);
						store_y[indice_points] = spline_y;
						store_x[indice_points] = half_points;
						store_thetas[indice_points] = carmen_normalize_theta(atan2(spline_y2 - spline_y, points_dx));
						indice_points++;
					}
				}
				half_points += acresc_points;
			}


			//double ref_theta = -1 * (globalpos.theta /*- preds[index].dtheta*/);
			//double ref_x = -1 * sqrt(globalpos.x * globalpos.x + globalpos.y * globalpos.y) * cos(atan2(globalpos.y, globalpos.x) + ref_theta);
			//double ref_y = -1 *(sqrt(globalpos.x * globalpos.x + globalpos.y * globalpos.y) * sin(atan2(globalpos.y, globalpos.x) + ref_theta)) + preds[index].dy;
			//SE2 ref_pose(ref_x, ref_y, ref_theta);

			std::vector<carmen_ackerman_traj_point_t> carmen_rddf_poses_from_spline_vec;
			std::vector<carmen_ackerman_traj_point_t> carmen_rddf_poses_local_vec;
			for (int i=0; i<indice_points; i++)
			{
				carmen_ackerman_traj_point_t waypoint_local;
				carmen_ackerman_traj_point_t waypoint;
				waypoint_local.x = store_x[i];
				waypoint_local.y = store_y[i];
				waypoint_local.theta = store_thetas[i];
				waypoint_local.v = 9.0;
				waypoint_local.phi = 0.2;
				carmen_rddf_poses_local_vec.push_back(waypoint_local);
				//SE2 pose_in_rddf_reference(i*0.5, store_points[i], store_thetas[i]);
				//SE2 pose_in_world_reference = ref_pose.inverse() * pose_in_rddf_reference;
				//waypoint.x = pose_in_world_reference[0];
				waypoint.x = globalpos.x + waypoint_local.x * cos(globalpos.theta) - (waypoint_local.y - preds[index].dy) * sin(globalpos.theta);
				//waypoint.y = pose_in_world_reference[1];
				waypoint.y = globalpos.y + waypoint_local.x * sin(globalpos.theta) + (waypoint_local.y - preds[index].dy) * cos(globalpos.theta);
				//waypoint.theta = carmen_normalize_theta(pose_in_world_reference[2]);
				waypoint.theta = waypoint_local.theta + globalpos.theta;
				waypoint.v = 9.0;
				waypoint.phi = 0.2;
				carmen_rddf_poses_from_spline_vec.push_back(waypoint);
			}
			carmen_point_t local_pos;
			local_pos.x = 0.0;
			local_pos.y = preds[index].dy;
			plot_state(carmen_rddf_poses_local_vec, local_pos);
			carmen_ackerman_traj_point_t *carmen_rddf_poses_from_spline = &carmen_rddf_poses_from_spline_vec[0];
//			for (int i = 0; i<indice_points; i++)
//			{
//				printf("%f %f\n", carmen_rddf_poses_from_spline[i].x, carmen_rddf_poses_from_spline[i].y);
//			}

			int annotations[2] = {1, 2};
			int annotation_codes[2] = {1, 2};

			carmen_rddf_publish_road_profile_message(
				carmen_rddf_poses_from_spline,
				&last_pose,
				indice_points,
				1,
				annotations,
				annotation_codes);
		}
		last_pose.x = globalpos.x;
		last_pose.y = globalpos.y;
		last_pose.theta = globalpos.theta;
	}
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
//	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, (char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT,
//					NULL, sizeof (carmen_behavior_selector_road_profile_message), (carmen_handler_t) rddf_handler, CARMEN_SUBSCRIBE_LATEST);
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

	std::ifstream input("preds_20190915.txt");
	Prediction s;
	while (input >> s)
	{
		preds.push_back(s);
	}

	std::sort(preds.begin(),
		  preds.end(),
		  compareByLength);

	for(int i=0; i<preds.size(); i++)
	{
		printf("%f\n", preds[i].timestamp);
	}

	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	read_parameters(argv);

	printf("Aguardando mensagem\n");

	subscribe_messages();

	carmen_ipc_dispatch();

	return 0;
}
