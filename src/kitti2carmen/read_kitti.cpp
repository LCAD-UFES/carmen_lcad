
#include <tf.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <carmen/carmen.h>
#include "read_kitti.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

carmen_velodyne_variable_scan_message
read_velodyne(char *dir, int file_id, double timestamp)
{
	FILE *stream;
	double x, y, z;
	double h, v, r;
	static char filename[1024];
	carmen_velodyne_variable_scan_message velodyne_message;

	sprintf(filename, "%s/%010d.bin", dir, file_id);

	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	int32_t num = 1000000;
	float* data = (float*) malloc(num * sizeof(float));

	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;

	stream = fopen(filename, "rb");

	num = fread(data, sizeof(float), num, stream) / 4;

	velodyne_message.number_of_shots = num / 64;
	velodyne_message.partial_scan = (carmen_velodyne_shot *) malloc (velodyne_message.number_of_shots * sizeof(carmen_velodyne_shot));
	velodyne_message.timestamp = timestamp;
	velodyne_message.host = (char*) "carmen";

	// angulo horizontal minimo
	double hmin = -M_PI + 0.0001;
	// angulo horizontal maximo
	double hmax = M_PI - 0.0001;
	// intervalo angular horizontal
	double hstep = (hmax - hmin) / (double) velodyne_message.number_of_shots;

	// angulo vertical minimo
	double vmin =  -0.45;
	// angulo vertical maximo
	double vmax = 0.111;
	// intervalo angular vertical
	double vstep = (vmax - vmin) / 64.0;

	// alocacao
	for (int i = 0; i < velodyne_message.number_of_shots; i++)
	{
		velodyne_message.partial_scan[i].shot_size = 64;
		velodyne_message.partial_scan[i].distance = (unsigned short*) calloc (64, sizeof(unsigned short));
		velodyne_message.partial_scan[i].intensity = (unsigned char*) calloc (64, sizeof(unsigned char));
		velodyne_message.partial_scan[i].angle = carmen_radians_to_degrees(carmen_normalize_theta(hmin + i * hstep));
	}

	double max = 0, min = 999;

	// preenchimento
	for (int i = 0; i < num; i++)
	{
		x = *px;
		y = *py;
		z = *pz;

		r = sqrt(x * x + y * y + z * z);
		v = asin(z / r);
		h = atan2(y, x);

		if (v > max) max = v;
		if (v < min) min = v;

		int h_id = (int) ((h - hmin) / hstep - 0.5);
		int v_id = (int) ((v - vmin) / vstep - 0.5);

		h_id = velodyne_message.number_of_shots - h_id - 1;

		if (h_id < 0 || h_id >= velodyne_message.number_of_shots || v_id < 0 || v_id >= 64)
		{
			printf("ERRO %lf %lf %lf %lf %lf %lf %d %d %d\n", v, h, r, x, y, z, h_id, v_id, velodyne_message.number_of_shots);
		}
		else
		{
			velodyne_message.partial_scan[h_id].distance[v_id] = (unsigned short) (r * 500.0);
			velodyne_message.partial_scan[h_id].intensity[v_id] = (unsigned char) ((*pr) * 255.0);
		}

		//cloud->points[i].rgba = *pr;
		px += 4;
		py += 4;
		pz += 4;
		pr += 4;
	}

	//printf("max: %lf min: %lf diff: %lf\n", max, min,max-min);
	free(data);
	fclose(stream);
	return velodyne_message;
}


carmen_bumblebee_basic_stereoimage_message
read_camera(char *left_dir, char *right_dir, int file_id, double timestamp)
{
	static char left_filename[1024];
	static char right_filename[1024];

	carmen_bumblebee_basic_stereoimage_message camera_message;

	sprintf(left_filename, "%s/%010d.png", left_dir, file_id);
	sprintf(right_filename, "%s/%010d.png", right_dir, file_id);

	printf("%s %s\n", left_filename, right_filename);

	// ler imagem
	cv::Mat o_left = cv::imread(left_filename);
	cv::Mat o_right = cv::imread(right_filename);

	cv::Mat left = cv::Mat(480, 640, CV_8UC3);
	cv::Mat right = cv::Mat(480, 640, CV_8UC3);

	cv::resize(o_left, left, left.size());
	cv::resize(o_right, right, right.size());

	// preencher mensagem do carmen
	camera_message.width = left.cols;
	camera_message.height = left.rows;
	camera_message.image_size = left.rows * left.cols * 3;
	camera_message.isRectified = 1;
	camera_message.raw_left = (unsigned char *) calloc (left.rows * left.cols * 3, sizeof(unsigned char));
	camera_message.raw_right = (unsigned char *) calloc (left.rows * left.cols * 3, sizeof(unsigned char));
	camera_message.timestamp = timestamp;
	camera_message.host = (char*) "carmen";

	for (int i = 0; i < left.rows; i++)
	{
		for (int j = 0; j < left.cols; j++)
		{
			camera_message.raw_left[3 * i * left.cols + 3 * j + 0] = left.data[i * left.step + 3 * j + 2];
			camera_message.raw_left[3 * i * left.cols + 3 * j + 1] = left.data[i * left.step + 3 * j + 1];
			camera_message.raw_left[3 * i * left.cols + 3 * j + 2] = left.data[i * left.step + 3 * j + 0];

			camera_message.raw_right[3 * i * left.cols + 3 * j + 0] = right.data[i * right.step + 3 * j + 2];
			camera_message.raw_right[3 * i * left.cols + 3 * j + 1] = right.data[i * right.step + 3 * j + 1];
			camera_message.raw_right[3 * i * left.cols + 3 * j + 2] = right.data[i * right.step + 3 * j + 0];
		}
	}

	return camera_message;
}

void
read_gps(char *dir_gps, int file_id, double timestamp, carmen_gps_gpgga_message *gps_msg,
		carmen_xsens_global_quat_message *xsens_msg)
{
	FILE *stream;
	static char filename[1024];

	double roll, pitch, yaw;
	double ignore_double;
	int ignore_int;

	sprintf(filename, "%s/%010d.txt", dir_gps, file_id);

	stream = fopen(filename, "rb");

	// GPS fields not present in kitti's log
	gps_msg->data_age = 0;
	gps_msg->geo_sea_level = 0;
	gps_msg->geo_sep = 0;
	gps_msg->gps_quality = 5; // qualidade alta
	gps_msg->hdop = 0;
	gps_msg->host = (char*) "carmen";
	gps_msg->lat_orient = 'N';
	gps_msg->latitude_dm = 0;
	gps_msg->long_orient = 'E';
	gps_msg->longitude_dm = 0;
	gps_msg->nr = 3;
	gps_msg->sea_level = 0;
	gps_msg->timestamp = timestamp;
	gps_msg->utc = 0;

	// XSENS fields not present in kitti's log
	xsens_msg->host = (char *) "carmen";
	xsens_msg->m_count = 0;
	xsens_msg->m_mag = {0, 0, 0};
	xsens_msg->m_temp = 0;
	xsens_msg->timestamp = timestamp;

	// gps
	fscanf(stream, "%lf", &(gps_msg->latitude));
	fscanf(stream, "%lf", &(gps_msg->longitude));
	fscanf(stream, "%lf", &(gps_msg->altitude));

	// ler xsens
	// roll, pitch, yaw
	fscanf(stream, "%lf", &(roll));
	fscanf(stream, "%lf", &(pitch));
	fscanf(stream, "%lf", &(yaw));

	tf::Quaternion q = tf::Quaternion(yaw, pitch, roll);

	xsens_msg->quat_data.m_data[0] = q.w();
	xsens_msg->quat_data.m_data[1] = q.x();
	xsens_msg->quat_data.m_data[2] = q.y();
	xsens_msg->quat_data.m_data[3] = q.z();

	// velocidades
	fscanf(stream, "%lf", &(ignore_double));
	fscanf(stream, "%lf", &(ignore_double));
	fscanf(stream, "%lf", &(ignore_double));
	fscanf(stream, "%lf", &(ignore_double));
	fscanf(stream, "%lf", &(ignore_double));

	// aceleracoes
	fscanf(stream, "%lf", &(xsens_msg->m_acc.x));
	fscanf(stream, "%lf", &(xsens_msg->m_acc.y));
	fscanf(stream, "%lf", &(xsens_msg->m_acc.z));
	fscanf(stream, "%lf", &(ignore_double));
	fscanf(stream, "%lf", &(ignore_double));

	// velocidade angular
	fscanf(stream, "%lf", &(xsens_msg->m_gyr.x));
	fscanf(stream, "%lf", &(xsens_msg->m_gyr.y));
	fscanf(stream, "%lf", &(xsens_msg->m_gyr.z));
	fscanf(stream, "%lf", &(ignore_double));
	fscanf(stream, "%lf", &(ignore_double));

	// pos and vel accuracy
	fscanf(stream, "%lf", &(ignore_double));
	fscanf(stream, "%lf", &(ignore_double));

	// information params
	fscanf(stream, "%d", &(ignore_int));
	fscanf(stream, "%d", &(gps_msg->num_satellites));
	fscanf(stream, "%d", &(ignore_int));
	fscanf(stream, "%d", &(ignore_int));
	fscanf(stream, "%d", &(ignore_int));

	printf("XSENS: %lf %lf %lf\n", roll, pitch, yaw);
	printf("GPS: %lf %lf\n", gps_msg->latitude, gps_msg->longitude);

	fclose(stream);
}


