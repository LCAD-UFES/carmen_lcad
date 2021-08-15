
/**
 * @description
 * DNN Global Localizer
 *
 * @author Alberto F. De Souza
 */

#include <stdio.h>
#include <string.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>

#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/web_cam_interface.h>
#include <carmen/stereo_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/xsens_interface.h>
#include <carmen/gps_xyz_messages.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/camera_drivers_interface.h>

#include "network.h"
#include "parser.h"

using namespace std;
using namespace cv;

static int camera;
static int bumblebee_basic_width;
static int bumblebee_basic_height;

network net;
char **learned_poses;
int last_correct_prediction = -1;
int contador = 0;
/****
 * usados para crop_image()
 *  esquerda  topo	   largura     altura		****/
int delta_x, delta_y, crop_width, crop_height;

/***
 * usados para lidar
 *  usar lidar   angulo a esquerda   angulo a direite
*/

int use_lidar, angle_left, angle_right;

cv::Mat
convert_darknet_image_to_cv_mat(image img)
{
	int channels = img.c;
	int width = img.w;
	int height = img.h;
	cv::Mat mat = cv::Mat(height, width, CV_8UC(channels));
	int step = mat.step;

	for (int y = 0; y < img.h; ++y)
	{
		for (int x = 0; x < img.w; ++x)
		{
			for (int c = 0; c < img.c; ++c)
			{
				float val = img.data[c * img.h * img.w + y * img.w + x];
				mat.data[y * step + x * img.c + c] = (unsigned char)(val * 255);
			}
		}
	}

	if (mat.channels() == 3)
		cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
	else if (mat.channels() == 4)
		cv::cvtColor(mat, mat, cv::COLOR_RGBA2BGR);

	return mat;
}

image convert_image_msg_to_darknet_image(unsigned int w, unsigned int h, unsigned char *data)
{
	unsigned int c = 3; // Number of channels
	image image = make_image(w, h, c);

	if (data == NULL)
		return image;

	for (unsigned int k = 0; k < c; ++k)
	{
		for (unsigned int j = 0; j < h; ++j)
		{
			for (unsigned int i = 0; i < w; ++i)
			{
				int dst_index = i + (w * j) + (w * h * k);
				int src_index = k + (c * i) + (c * w * j);
				image.data[dst_index] = (float)(data[src_index] / 255.0); // 255 because of conversion Uchar (byte) ti float
			}
		}
	}

	return (image);
}

double
infer_pose(carmen_point_t *pose, double width, double height,
		   int dx, int dy, int w, int h,
		   unsigned char *image_raw, double timestamp)
{
	image img = convert_image_msg_to_darknet_image(width, height, image_raw);
	free(image_raw);
	image img_without_car_hood = crop_image(img, dx, dy, w, h); // crop_image() nao faz free()
	image resized_img_without_car_hood = resize_min(img_without_car_hood, net.w);
	image cropped_resized_img_without_car_hood = crop_image(resized_img_without_car_hood, (resized_img_without_car_hood.w - net.w) / 2, (resized_img_without_car_hood.h - net.h) / 2, net.w, net.h);

	cv::Mat mat = convert_darknet_image_to_cv_mat(cropped_resized_img_without_car_hood);
	cv::namedWindow("Cropped Image", cv::WINDOW_NORMAL);
	cv::imshow("Cropped Image", mat);

	float *predictions = network_predict(net, cropped_resized_img_without_car_hood.data);

	int selected_pose_label;
	top_k(predictions, net.outputs, 1, &selected_pose_label);

	//verifica se é a primeira pose detectada
	if (last_correct_prediction == -1)
	{
		last_correct_prediction = selected_pose_label; //inicializa last_correct_prediction
	}
	else if ((selected_pose_label >= last_correct_prediction) && (selected_pose_label <= (last_correct_prediction + 10)))
	{												   //verifica se está dentro da MAE
		last_correct_prediction = selected_pose_label; //atualiza last_correct_prediction
		contador = 0;
	}
	else
	{
		selected_pose_label = last_correct_prediction; // caso negativo, pega a última pose
		contador++;
		if (contador > 10)
			last_correct_prediction = -1;
	}

	char predicted_image_file_name[2048];
	sscanf(learned_poses[selected_pose_label], "%lf %lf %lf %s", &(pose->x), &(pose->y), &(pose->theta), predicted_image_file_name);
	printf("confidence %lf, %lf %lf %lf %s\n", predictions[selected_pose_label], pose->x, pose->y, pose->theta, predicted_image_file_name);
	//	printf("confidence %lf, %s\n", predictions[selected_pose_label], learned_poses[selected_pose_label]);

	Mat pose_image = imread(predicted_image_file_name, IMREAD_COLOR);
	if (predictions[selected_pose_label] < 0.00)
		pose_image = Mat::zeros(Size(pose_image.cols, pose_image.rows), pose_image.type());
	imshow("dnn_visual_gl", pose_image);
	waitKey(1);

	free_image(img);
	free_image(img_without_car_hood);
	free_image(resized_img_without_car_hood);
	free_image(cropped_resized_img_without_car_hood);

	return (predictions[selected_pose_label]);
}

void carmen_gps_xyz_publish_message(carmen_gps_xyz_message gps_xyz_message)
{
	IPC_RETURN_TYPE err = IPC_OK;

	err = IPC_publishData(CARMEN_GPS_XYZ_MESSAGE_NAME, &gps_xyz_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_GPS_XYZ_MESSAGE_NAME);
}

void publish_carmen_gps_gphdt_message(carmen_gps_gphdt_message *carmen_extern_gphdt_ptr)
{
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_extern_gphdt_ptr != NULL)
	{
		err = IPC_publishData(CARMEN_GPS_GPHDT_MESSAGE_NAME, carmen_extern_gphdt_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPHDT_MESSAGE_NAME);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void publish_gps_xyz(double x, double y, double theta, double confidence, double timestamp)
{
	carmen_gps_xyz_message gps_xyz_message = {};

	gps_xyz_message.nr = 1; // Trimble
							//	gps_xyz_message.utc = gps_gpgga->utc;
							//	gps_xyz_message.latitude = gps_gpgga->latitude;
							//	gps_xyz_message.latitude_dm = gps_gpgga->latitude_dm;
							//	gps_xyz_message.lat_orient = gps_gpgga->lat_orient;
							//	gps_xyz_message.longitude = gps_gpgga->longitude;
							//	gps_xyz_message.longitude_dm = gps_gpgga->longitude_dm;
							//	gps_xyz_message.long_orient = gps_gpgga->long_orient;
	if (confidence > 0.1)
		gps_xyz_message.gps_quality = 4;
	else
		gps_xyz_message.gps_quality = 0;
	//	gps_xyz_message.num_satellites = gps_gpgga->num_satellites;
	//	gps_xyz_message.hdop = gps_gpgga->hdop;
	//	gps_xyz_message.sea_level = gps_gpgga->sea_level;
	//	gps_xyz_message.altitude = gps_gpgga->altitude;
	//	gps_xyz_message.geo_sea_level = gps_gpgga->geo_sea_level;
	//	gps_xyz_message.geo_sep = gps_gpgga->geo_sep;
	//	gps_xyz_message.data_age = gps_gpgga->data_age;

	//	if (gps_gpgga->lat_orient == 'S') latitude = -gps_gpgga->latitude;
	//	if (gps_gpgga->long_orient == 'W') longitude = -gps_gpgga->longitude;

	gps_xyz_message.x = x;
	gps_xyz_message.y = y;
	gps_xyz_message.z = 0.0;

	gps_xyz_message.timestamp = timestamp;
	gps_xyz_message.host = carmen_get_host();

	carmen_gps_xyz_publish_message(gps_xyz_message);

	carmen_gps_gphdt_message carmen_gphdt;
	carmen_gphdt.nr = 1;
	carmen_gphdt.heading = theta;
	if (confidence > 0.1)
		carmen_gphdt.valid = 1;
	else
		carmen_gphdt.valid = 0;
	carmen_gphdt.timestamp = timestamp;
	carmen_gphdt.host = carmen_get_host();

	publish_carmen_gps_gphdt_message(&carmen_gphdt);
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void velodyne_partial_scan_handler(carmen_velodyne_partial_scan_message *lidar)
{
	carmen_point_t pose;

	double timestamp = lidar->timestamp;
	//total de colunas
	int shots = lidar->number_of_32_laser_shots;
	// coletar a partir da coluna ini
	int ini = (int)((float)(shots / 360) * (180 - abs(angle_left)));
	// coletar ate a coluna end
	int end = (int)((float)(shots / 360) * (180 + abs(angle_right)));
	// total de linhas
	int vertical_resolution = 32;
	// total de pixels        total de linhas  *  colunas usadas
	int number_of_pixels = vertical_resolution * (end - ini);

	double range[32];
	double by500 = 1 / 500;
	double by25 = 1 / 25;

	Mat synthetic_image;
    synthetic_image = Mat::zeros(Size((end-ini),32), CV_8UC1);
	int R,G,B;

	// fiz assim para evitar cascateamento de loop, pois eram poucas linhas
	for (int i = ini; i < end; i++)
	{
		range[0] = ((int)((lidar->partial_scan[i].distance[31]) * by500) * 765 * by25);
		B = 0 ? range[0] < 511 : range[0] - 510;
		G = 0 ? range[0] < 256 : range[0] - B - 255;
		R = 255 ? range[0] > 255 : range[0];
		if( range[0] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,0))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,0))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,0))[2] = R;
		range[1] = ((int)((lidar->partial_scan[i].distance[29]) * by500) * 765 * by25);
		B = 0 ? range[1] < 511 : range[1] - 510;
		G = 0 ? range[1] < 256 : range[1] - B - 255;
		R = 255 ? range[1] > 255 : range[1];
		if( range[1] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,1))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,1))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,1))[2] = R;
		range[2] = ((int)((lidar->partial_scan[i].distance[27]) * by500) * 765 * by25);
		B = 0 ? range[2] < 511 : range[2] - 510;
		G = 0 ? range[2] < 256 : range[2] - B - 255;
		R = 255 ? range[2] > 255 : range[2];
		if( range[2] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,2))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,2))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,2))[2] = R;
		range[3] = ((int)((lidar->partial_scan[i].distance[25]) * by500) * 765 * by25);
		B = 0 ? range[3] < 511 : range[3] - 510;
		G = 0 ? range[3] < 256 : range[3] - B - 255;
		R = 255 ? range[3] > 255 : range[3];
		if( range[3] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,3))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,3))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,3))[2] = R;
		range[4] = ((int)((lidar->partial_scan[i].distance[23]) * by500) * 765 * by25);
		B = 0 ? range[4] < 511 : range[4] - 510;
		G = 0 ? range[4] < 256 : range[4] - B - 255;
		R = 255 ? range[4] > 255 : range[4];
		if( range[4] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,4))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,4))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,4))[2] = R;
		range[5] = ((int)((lidar->partial_scan[i].distance[21]) * by500) * 765 * by25);
		B = 0 ? range[5] < 511 : range[5] - 510;
		G = 0 ? range[5] < 256 : range[5] - B - 255;
		R = 255 ? range[5] > 255 : range[5];
		if( range[5] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,5))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,5))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,5))[2] = R;
		range[6] = ((int)((lidar->partial_scan[i].distance[19]) * by500) * 765 * by25);
		B = 0 ? range[6] < 511 : range[6] - 510;
		G = 0 ? range[6] < 256 : range[6] - B - 255;
		R = 255 ? range[6] > 255 : range[6];
		if( range[6] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,6))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,6))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,6))[2] = R;
		range[7] = ((int)((lidar->partial_scan[i].distance[17]) * by500) * 765 * by25);
		B = 0 ? range[7] < 511 : range[7] - 510;
		G = 0 ? range[7] < 256 : range[7] - B - 255;
		R = 255 ? range[7] > 255 : range[7];
		if( range[7] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,7))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,7))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,7))[2] = R;
		range[8] = ((int)((lidar->partial_scan[i].distance[15]) * by500) * 765 * by25);
		B = 0 ? range[8] < 511 : range[8] - 510;
		G = 0 ? range[8] < 256 : range[8] - B - 255;
		R = 255 ? range[8] > 255 : range[8];
		if( range[8] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,8))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,8))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,8))[2] = R;
		range[9] = ((int)((lidar->partial_scan[i].distance[13]) * by500) * 765 * by25);
		B = 0 ? range[9] < 511 : range[9] - 510;
		G = 0 ? range[9] < 256 : range[9] - B - 255;
		R = 255 ? range[9] > 255 : range[9];
		if( range[9] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,9))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,9))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,9))[2] = R;
		range[10] = ((int)((lidar->partial_scan[i].distance[11]) * by500) * 765 * by25);
		B = 0 ? range[10] < 511 : range[10] - 510;
		G = 0 ? range[10] < 256 : range[10] - B - 255;
		R = 255 ? range[10] > 255 : range[10];
		if( range[10] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,10))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,10))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,10))[2] = R;
		range[11] = ((int)((lidar->partial_scan[i].distance[9]) * by500) * 765 * by25);
		B = 0 ? range[11] < 511 : range[11] - 510;
		G = 0 ? range[11] < 256 : range[11] - B - 255;
		R = 255 ? range[11] > 255 : range[11];
		if( range[11] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,11))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,11))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,11))[2] = R;
		range[12] = ((int)((lidar->partial_scan[i].distance[7]) * by500) * 765 * by25);
		B = 0 ? range[12] < 511 : range[12] - 510;
		G = 0 ? range[12] < 256 : range[12] - B - 255;
		R = 255 ? range[12] > 255 : range[12];
		if( range[12] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,12))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,12))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,12))[2] = R;
		range[13] = ((int)((lidar->partial_scan[i].distance[5]) * by500) * 765 * by25);
		B = 0 ? range[13] < 511 : range[13] - 510;
		G = 0 ? range[13] < 256 : range[13] - B - 255;
		R = 255 ? range[13] > 255 : range[13];
		if( range[13] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,13))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,13))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,13))[2] = R;
		range[14] = ((int)((lidar->partial_scan[i].distance[3]) * by500) * 765 * by25);
		B = 0 ? range[14] < 511 : range[14] - 510;
		G = 0 ? range[14] < 256 : range[14] - B - 255;
		R = 255 ? range[14] > 255 : range[14];
		if( range[14] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,14))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,14))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,14))[2] = R;
		range[15] = ((int)((lidar->partial_scan[i].distance[1]) * by500) * 765 * by25);
		B = 0 ? range[15] < 511 : range[15] - 510;
		G = 0 ? range[15] < 256 : range[15] - B - 255;
		R = 255 ? range[15] > 255 : range[15];
		if( range[15] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,151))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,151))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,151))[2] = R;
		range[16] = ((int)((lidar->partial_scan[i].distance[30]) * by500) * 765 * by25);
		B = 0 ? range[16] < 511 : range[16] - 510;
		G = 0 ? range[16] < 256 : range[16] - B - 255;
		R = 255 ? range[16] > 255 : range[16];
		if( range[16] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,16))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,16))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,16))[2] = R;
		range[17] = ((int)((lidar->partial_scan[i].distance[28]) * by500) * 765 * by25);
		B = 0 ? range[17] < 511 : range[17] - 510;
		G = 0 ? range[17] < 256 : range[17] - B - 255;
		R = 255 ? range[17] > 255 : range[17];
		if( range[17] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,17))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,17))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,17))[2] = R;
		range[18] = ((int)((lidar->partial_scan[i].distance[26]) * by500) * 765 * by25);
		B = 0 ? range[18] < 511 : range[18] - 510;
		G = 0 ? range[18] < 256 : range[18] - B - 255;
		R = 255 ? range[18] > 255 : range[18];
		if( range[18] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,18))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,18))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,18))[2] = R;
		range[19] = ((int)((lidar->partial_scan[i].distance[24]) * by500) * 765 * by25);
		B = 0 ? range[19] < 511 : range[19] - 510;
		G = 0 ? range[19] < 256 : range[19] - B - 255;
		R = 255 ? range[19] > 255 : range[19];
		if( range[19] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,19))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,19))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,19))[2] = R;
		range[20] = ((int)((lidar->partial_scan[i].distance[22]) * by500) * 765 * by25);
		B = 0 ? range[20] < 511 : range[20] - 510;
		G = 0 ? range[20] < 256 : range[20] - B - 255;
		R = 255 ? range[20] > 255 : range[20];
		if( range[20] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,20))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,20))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,20))[2] = R;
		range[21] = ((int)((lidar->partial_scan[i].distance[20]) * by500) * 765 * by25);
		B = 0 ? range[21] < 511 : range[21] - 510;
		G = 0 ? range[21] < 256 : range[21] - B - 255;
		R = 255 ? range[21] > 255 : range[21];
		if( range[21] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,21))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,21))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,21))[2] = R;
		range[22] = ((int)((lidar->partial_scan[i].distance[18]) * by500) * 765 * by25);
		B = 0 ? range[22] < 511 : range[22] - 510;
		G = 0 ? range[22] < 256 : range[22] - B - 255;
		R = 255 ? range[22] > 255 : range[22];
		if( range[22] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,22))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,22))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,22))[2] = R;
		range[23] = ((int)((lidar->partial_scan[i].distance[16]) * by500) * 765 * by25);
		B = 0 ? range[23] < 511 : range[23] - 510;
		G = 0 ? range[23] < 256 : range[23] - B - 255;
		R = 255 ? range[23] > 255 : range[23];
		if( range[23] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,23))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,23))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,23))[2] = R;
		range[24] = ((int)((lidar->partial_scan[i].distance[14]) * by500) * 765 * by25);
		B = 0 ? range[24] < 511 : range[24] - 510;
		G = 0 ? range[24] < 256 : range[24] - B - 255;
		R = 255 ? range[24] > 255 : range[24];
		if( range[24] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,24))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,24))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,24))[2] = R;
		range[25] = ((int)((lidar->partial_scan[i].distance[12]) * by500) * 765 * by25);
		B = 0 ? range[25] < 511 : range[25] - 510;
		G = 0 ? range[25] < 256 : range[25] - B - 255;
		R = 255 ? range[25] > 255 : range[25];
		if( range[25] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,25))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,25))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,25))[2] = R;
		range[26] = ((int)((lidar->partial_scan[i].distance[10]) * by500) * 765 * by25);
		B = 0 ? range[26] < 511 : range[26] - 510;
		G = 0 ? range[26] < 256 : range[26] - B - 255;
		R = 255 ? range[26] > 255 : range[26];
		if( range[26] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,26))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,26))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,26))[2] = R;
		range[27] = ((int)((lidar->partial_scan[i].distance[8]) * by500) * 765 * by25);
		B = 0 ? range[27] < 511 : range[27] - 510;
		G = 0 ? range[27] < 256 : range[27] - B - 255;
		R = 255 ? range[27] > 255 : range[27];
		if( range[27] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,27))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,27))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,27))[2] = R;
		range[28] = ((int)((lidar->partial_scan[i].distance[6]) * by500) * 765 * by25);
		B = 0 ? range[28] < 511 : range[28] - 510;
		G = 0 ? range[28] < 256 : range[28] - B - 255;
		R = 255 ? range[28] > 255 : range[28];
		if( range[28] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,28))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,28))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,28))[2] = R;
		range[29] = ((int)((lidar->partial_scan[i].distance[4]) * by500) * 765 * by25);
		B = 0 ? range[29] < 511 : range[29] - 510;
		G = 0 ? range[29] < 256 : range[29] - B - 255;
		R = 255 ? range[29] > 255 : range[29];
		if( range[29] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,29))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,29))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,29))[2] = R;
		range[30] = ((int)((lidar->partial_scan[i].distance[2]) * by500) * 765 * by25);
		B = 0 ? range[30] < 511 : range[30] - 510;
		G = 0 ? range[30] < 256 : range[30] - B - 255;
		R = 255 ? range[30] > 255 : range[30];
		if( range[30] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,30))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,30))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,30))[2] = R;
		range[31] = ((int)((lidar->partial_scan[i].distance[0]) * by500) * 765 * by25);
		B = 0 ? range[31] < 511 : range[31] - 510;
		G = 0 ? range[31] < 256 : range[31] - B - 255;
		R = 255 ? range[31] > 255 : range[31];
		if( range[31] == 0) B = G = R = 255;
		synthetic_image.at<Vec3b>(Point(i,31))[0] = B;
		synthetic_image.at<Vec3b>(Point(i,31))[1] = G;
		synthetic_image.at<Vec3b>(Point(i,31))[2] = R;
		
	}
	
	//resize de (shots x 32) para o (crop_width x crop_height)
	resize(synthetic_image,synthetic_image,Size(crop_width, crop_height),0,0, CV_INTER_LINEAR);
	// inferir pose
	double confidence = infer_pose(&pose, crop_width, crop_height,
									0, 0, 640, 380,
									synthetic_image.data, lidar->timestamp);
	// publicar pose
	publish_gps_xyz(pose.x, pose.y, pose.theta, confidence, lidar->timestamp);
	// liberar memoria
	free(synthetic_image.data);
}

void bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	carmen_point_t pose;
	double confidence = infer_pose(&pose, stereo_image->width, stereo_image->height,
								   0, 0, 640, 380,
								   stereo_image->raw_right, stereo_image->timestamp);

	publish_gps_xyz(pose.x, pose.y, pose.theta, confidence, stereo_image->timestamp);
}

void camera_drivers_message_handler(camera_message *msg)
{
	carmen_point_t pose;
	double confidence = infer_pose(&pose, msg->images[0].width, msg->images[0].height,
								   0, 50, 640, 380,
								   (unsigned char *)msg->images[0].raw_data, msg->timestamp);

	publish_gps_xyz(pose.x, pose.y, pose.theta, confidence, msg->timestamp);
}

void shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("log_filter: disconnected\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

void read_parameters(int argc, char **argv)
{
	char bumblebee_string[256];
	char camera_string[256];

	carmen_param_t param_cmd_list[] =
		{
			{(char *)"commandline", (char *)"camera_id", CARMEN_PARAM_INT, &camera, 0, NULL},
		};

	carmen_param_install_params(argc, argv, param_cmd_list, sizeof(param_cmd_list) / sizeof(param_cmd_list[0]));

	sprintf(camera_string, "%s%d", "camera", camera);
	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

	carmen_param_t param_list[] =
		{
			{bumblebee_string, (char *)"width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
			{bumblebee_string, (char *)"height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL},
		};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

void initialize_structures(char *cfgfile, char *weightfile, char *learned_poses_filename, int dx, int dy, int w, int h, int use_ldr, int angle_lft, int angle_rgt)
{
	delta_x = dx;
	delta_y = dy;
	crop_width = w;
	crop_height = h;
	use_lidar = use_ldr ? use_ldr : 0;
	angle_left = angle_lft ? std::abs(angle_lft) : 45;
	angle_right = angle_rgt ? std::abs(angle_rgt) : 45;
	net = parse_network_cfg_custom(cfgfile, 1, 0);
	if (weightfile)
		load_weights(&net, weightfile);

	set_batch_network(&net, 1);
	srand(2222222);

	fuse_conv_batchnorm(net);
	calculate_binary_weights(net);

	learned_poses = get_labels(learned_poses_filename);
}

void subscribe_messages()
{
	if (use_lidar == 1)
	{
		carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t)velodyne_partial_scan_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t)bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(camera, NULL, (carmen_handler_t)camera_drivers_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
	if (argc != 13)
	{
		printf(" Usage: ./deep_vgl config/config.cfg config/classifier.weights config/poses_and_labels.txt 0 0 640 480 0 45 45 -camera_id 3\n");
		exit(1);
	}

	signal(SIGINT, shutdown_module);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	initialize_structures(argv[1], argv[2], argv[3], atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]), atoi(argv[8]), atoi(argv[9]), atoi(argv[10]));

	read_parameters(argc, argv);

	subscribe_messages();
	carmen_ipc_dispatch();

	return (0);
}
