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
char save_buffer[10000];
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
/*
vector<carmen_position_t>
get_rddf_points_in_image_full (tf::StampedTransform world_to_camera_pose, int img_width, int img_height)
{
	carmen_position_t p;
	vector<carmen_position_t> rddf_points_in_image;
	double distance, last_distance;
	for(int i = 0; i < last_rddf_poses.number_of_poses; i++)
	{
		p = convert_rddf_pose_to_point_in_image (last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, 0.0, world_to_camera_pose, camera_parameters, img_width, img_height);
		rddf_points_in_image.push_back(p);
	}
	return (rddf_points_in_image);
}
*/
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Subscribes                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc , char **argv)
{
	std::ifstream input("log_20191002.txt");
	Prediction s;
	while (input >> s)
	{
		preds.push_back(s);
	}

	std::sort(preds.begin(),
		  preds.end(),
		  compareByLength);
	memset(save_buffer,'\0', 10000*sizeof(char));
	memset(txt_name,'\0',150*sizeof(char));
	struct stat sb;
	if(!(stat("/dados/rddf_predict", &sb) == 0))
		mkdir("/dados/rddf_predict",0777);
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	snprintf(txt_name, sizeof(txt_name), "/dados/rddf_predict/rddf_spline_points_%d-%d-%d_%d:%d:%d",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	file_log = fopen(txt_name, "a");
	if(NULL == file_log)
	{
		printf("Erro ao abrir o arquivo log_rddf_predict.txt no método main (rddf_metrics.c)\n");
		exit(1);
	}


	for(int i=0; i<preds.size(); i++)
	{
		double timestamp = preds[i].timestamp;
		double dy = preds[i].dy;
		double k1 = preds[i].k1;
		double k2 = preds[i].k2;
		double k3 = preds[i].k3;

		gsl_interp_accel *acc;
		gsl_spline *phi_spline;
		double knots_x[4] = {0.0,  30/ 3.0, 2 * 30 / 3.0, 30.0};
		double knots_y[4] = {0.0, k1, k2, k3};
		acc = gsl_interp_accel_alloc();
		const gsl_interp_type *type = gsl_interp_cspline;
		phi_spline = gsl_spline_alloc(type, 4);
		gsl_spline_init(phi_spline, knots_x, knots_y, 4);

		double half_points = 0.0;
		double acresc_points = 0.01;
//		double store_points[int(30/acresc_points)+1];
		int indice_points = 0;

		snprintf(save_buffer, sizeof(save_buffer),"%f ", timestamp);
		fprintf(file_log,"%s", save_buffer);


		while( half_points <= 30.0 )
		{
			double spline_y = gsl_spline_eval(phi_spline, half_points, acc);
//			store_points[indice_points] = spline_y;
			indice_points++;
			printf("aaaa = %f\n", half_points);
			half_points += acresc_points;
			snprintf(save_buffer, sizeof(save_buffer),"%f ", spline_y - dy);
			fprintf(file_log,"%s", save_buffer);
		}

//		printf("aaaa = %d\n", indice_points);
		fprintf(file_log,"\n");


	}
	fclose(file_log);

	return 0;
}
