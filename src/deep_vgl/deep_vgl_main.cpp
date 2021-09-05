
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

//#pragma pack(push, 2)
struct RGB
{
	uchar B;
	uchar G;
	uchar R;
};

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


image
convert_image_msg_to_darknet_image(unsigned int w, unsigned int h, unsigned char *data)
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
	printf("infering pose\n");
	image img = convert_image_msg_to_darknet_image(width, height, image_raw);
	image img_without_car_hood = crop_image(img, dx, dy, w, h); // crop_image() nao faz free()
	int min = net.h < net.w ? net.h : net.w;
	image resized_img_without_car_hood = resize_min(img_without_car_hood, min);
	image cropped_resized_img_without_car_hood = crop_image(resized_img_without_car_hood, (resized_img_without_car_hood.w - net.w) / 2, (resized_img_without_car_hood.h - net.h) / 2, net.w, net.h);
	cv::Mat mat = convert_darknet_image_to_cv_mat(cropped_resized_img_without_car_hood);
	cv::namedWindow("Cropped Image", cv::WINDOW_NORMAL);
	cv::imshow("Cropped Image", mat);

	float *predictions = network_predict(net, cropped_resized_img_without_car_hood.data);

	int selected_pose_label;
	top_k(predictions, net.outputs, 1, &selected_pose_label);

	//verifica se é a primeira pose detectada
	// if (last_correct_prediction == -1)
	// {
	// 	last_correct_prediction = selected_pose_label; //inicializa last_correct_prediction
	// }
	// else if ((selected_pose_label >= last_correct_prediction) && (selected_pose_label <= (last_correct_prediction + 10)))
	// {												   //verifica se está dentro da MAE
	// 	last_correct_prediction = selected_pose_label; //atualiza last_correct_prediction
	// 	contador = 0;
	// }
	// else
	// {
	// 	selected_pose_label = last_correct_prediction; // caso negativo, pega a última pose
	// 	contador++;
	// 	if (contador > 10)
	// 		last_correct_prediction = -1;
	// }

	char predicted_image_file_name[2048];
	sscanf(learned_poses[selected_pose_label], "%lf %lf %lf %s", &(pose->x), &(pose->y), &(pose->theta), predicted_image_file_name);
	printf("confidence %lf, %lf %lf %lf %s\n", predictions[selected_pose_label], pose->x, pose->y, pose->theta, predicted_image_file_name);
	
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


void
carmen_gps_xyz_publish_message(carmen_gps_xyz_message gps_xyz_message)
{
	IPC_RETURN_TYPE err = IPC_OK;

	err = IPC_publishData(CARMEN_GPS_XYZ_MESSAGE_NAME, &gps_xyz_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_GPS_XYZ_MESSAGE_NAME);
}


void
publish_carmen_gps_gphdt_message(carmen_gps_gphdt_message *carmen_extern_gphdt_ptr)
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


void
publish_gps_xyz(double x, double y, double theta, double confidence, double timestamp)
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


void
velodyne_partial_scan_handler(carmen_velodyne_partial_scan_message *lidar)
{
	printf("acquiring velodyne message...\n");
	carmen_point_t pose;

	double timestamp = lidar->timestamp;
	//total de colunas
	int shots = lidar->number_of_32_laser_shots;
	if (shots < 1000)
		return;

	// coletar a partir da coluna ini
	int ini = (int)((float)(shots / 360) * (180 - abs(angle_left)));
	// coletar ate a coluna end
	int end = (int)((float)(shots / 360) * (180 + abs(angle_right)));
	// total de linhas
	int vertical_resolution = 32;
	// total de pixels        total de linhas  *  colunas usadas
	int number_of_pixels = vertical_resolution * (end - ini);

	double range[32];
	double by500 = (double)(1 / 500.0); // para poupar tempo no loop
	double by25 = (double)(1 / 25.0);	// para poupar tempo no loop

	Mat synthetic_image = Mat::zeros(Size((end - ini), 32), CV_8UC3);
	int ray_order[32] = {31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0};
	for (int r = 0; r < 32; r++)
	{
		cv::Vec3b* ptr = synthetic_image.ptr<cv::Vec3b>(r);
		for (int i = ini; i < end; i++)
		{
			int R, G, B;
			range[r] = ((int)((lidar->partial_scan[i].distance[ray_order[r]]) * by500) * 765 * by25);
			B = range[r] < 511 ? 0 : range[r] - 510;
			G = range[r] < 256 ? 0 : range[r] - B - 255;
			R = range[r] > 255 ? 255 : range[r];
			if (range[r] == 0)
				B = G = R = 255;
			ptr[i-ini] = Vec3b(R,G,B);
		}
	}
	
	//resize de (shots x 32) para o (crop_width x crop_height)
	Mat resized_image;
	cv::resize(synthetic_image,resized_image,Size(crop_width, crop_height), 1, 1, CV_INTER_AREA);
	
	// inferir pose
	double confidence = infer_pose(&pose, crop_width, crop_height,
								   0, 0, 640, 480,
								   resized_image.data, lidar->timestamp);
	// publicar pose
	publish_gps_xyz(pose.x, pose.y, pose.theta, confidence, lidar->timestamp);
	// liberar memoria
	//free(synthetic_image.data);
}


void
bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	carmen_point_t pose;
	double confidence = infer_pose(&pose, stereo_image->width, stereo_image->height,
								   0, 0, 640, 380,
								   stereo_image->raw_right, stereo_image->timestamp);

	publish_gps_xyz(pose.x, pose.y, pose.theta, confidence, stereo_image->timestamp);
}


void
camera_drivers_message_handler(camera_message *msg)
{
	carmen_point_t pose;
	double confidence = infer_pose(&pose, msg->images[0].width, msg->images[0].height,
								   0, 50, 640, 380,
								   (unsigned char *)msg->images[0].raw_data, msg->timestamp);

	publish_gps_xyz(pose.x, pose.y, pose.theta, confidence, msg->timestamp);
}


void
shutdown_module(int signo)
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


void
read_parameters(int argc, char **argv)
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


void
initialize_structures(char *cfgfile, char *weightfile, char *learned_poses_filename, int dx, int dy, int w, int h, int use_ldr, int angle_lft, int angle_rgt)
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
	printf("all done on darknet\n");
}


void
subscribe_messages()
{
	if (use_lidar == 1)
	{
		printf("subscribing deep_vgl on velodyne\n");
		carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t)velodyne_partial_scan_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t)bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(camera, NULL, (carmen_handler_t)camera_drivers_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char *argv[])
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
