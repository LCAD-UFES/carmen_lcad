#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/libadabins.h>
#include <carmen/libdpt.h>
#include <carmen/libglpdepth.h>
#include <carmen/libnewcrfs.h>
#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <carmen/camera_drivers_messages.h>
#include <carmen/camera_drivers_interface.h>
#include <carmen/param_interface.h>
#include <carmen/param_messages.h>

#include <carmen/velodyne_camera_calibration.h>
#include <fstream>
#include <tf.h>

/*Message stereo*/
#include <carmen/stereo_interface.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_point_cloud_interface.h>

using namespace std;
using namespace cv;

static int camera;
static int vertical_resolution = 480;
static int horizontal_resolution = 640;
static double range_max;
static double horizontal_camera_angle;
static double horizontal_start_angle;
static double vertical_camera_angle;
static double range_multiplier_factor;
static int vertical_top_cut;
static int vertical_down_cut;
static int lidar_num;

static int camera_width = 640;
static int camera_height = 480;

static carmen_velodyne_shot *scan;
static carmen_velodyne_variable_scan_message velodyne_partial_scan;

static IplImage *reference_image;
static IplImage *reference_image_gray;

/*Message stereo*/
static carmen_stereo_point_cloud_message point_cloud_message;
static stereo_util su;

Mat cameraMatrix, distCoeffs, newcameramtx, R1;
double fx, fy, cu, camera_cv, k1, k2, p1, p2, k3;
static Mat MapX, MapY;

static char **modules;
static int num_modules;

char *neural_network = (char *)"glpdepth";
unsigned char *depth_pred;

inline double round(double val)
{
	if (val < 0)
		return ceil(val - 0.5);
	return floor(val + 0.5);
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
/*Message stereo*/
static void 
register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_STEREO_POINT_CLOUD_NAME, IPC_VARIABLE_LENGTH, CARMEN_STEREO_POINT_CLOUD_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_STEREO_POINT_CLOUD_NAME);

}



void convert_depth_to_velodyne_beams(unsigned char *depth, int vertical_resolution,
									 int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
									 double range_max, unsigned char *image_gray, unsigned char *image_color)
{
	int i, j, x, y;

	unsigned short int *points = (unsigned short int *)depth;

	double angle = horizontal_start_angle; // Horizontal angle
	double delta_angle = horizontal_camera_angle / (double)horizontal_resolution;
	for (x = 0, j = horizontal_resolution - 1; x < horizontal_resolution; x += 1, j--)
	{
		stereo_velodyne_scan[j].angle = angle;
		angle += delta_angle;
		for (y = 0, i = vertical_resolution - 1; y < vertical_resolution; y += 1, i--)
		{
			// double horizontal_angle = carmen_normalize_theta(-stereo_velodyne_scan[j].angle) / 180.0;
			//  double horizontal_angle = stereo_velodyne_scan[j].angle * M_PI / 180.0;
			double horizontal_angle = carmen_normalize_theta(carmen_degrees_to_radians(stereo_velodyne_scan[j].angle)) / M_PI;
			double range = points[(int)(y * (double)horizontal_resolution + x)] * (2 - cos(abs(horizontal_angle)));
			range = range > range_max ? 0.0 : range;
			stereo_velodyne_scan[j].distance[i] = (unsigned short)(range * range_multiplier_factor);
			stereo_velodyne_scan[j].intensity[i] = image_gray[(int)(y * (double)horizontal_resolution + x)];
			// stereo_velodyne_scan[j].point_color[i].x = (double) image_color[(int)(3*(y * (double)horizontal_resolution + x))];
			// stereo_velodyne_scan[j].point_color[i].y = (double) image_color[(int)(3*(y * (double)horizontal_resolution + x)+1)];
			// stereo_velodyne_scan[j].point_color[i].z = (double) image_color[(int)(3*(y * (double)horizontal_resolution + x)+2)];
			// cout << stereo_velodyne_scan[j].point_color[i].x << endl;
		}
	}
	
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	Mat open_cv_image = Mat(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_left, 0);

	
	
	cv::Mat imggray;
	cv::cvtColor(open_cv_image, imggray, cv::COLOR_BGR2GRAY);
	unsigned char *image_gray = imggray.data;
	
	cv::cvtColor(open_cv_image, open_cv_image, cv::COLOR_BGR2RGB);
	if (!strcmp(neural_network, "adabins"))
		depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "dpt"))
		depth_pred = libdpt_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "glpdepth"))
		depth_pred = libglpdepth_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "newcrfs"))
		depth_pred = libnewcrfs_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);

	cv::Mat imgdepth = cv::Mat(open_cv_image.rows, open_cv_image.cols, CV_16U, depth_pred);
	unsigned char *image_color = open_cv_image.data;
	convert_depth_to_velodyne_beams(depth_pred, vertical_resolution,
									horizontal_resolution, scan,
									range_max, image_gray, image_color);

	velodyne_partial_scan.partial_scan = scan;
	velodyne_partial_scan.number_of_shots = horizontal_resolution;
	velodyne_partial_scan.host = carmen_get_host();
	velodyne_partial_scan.timestamp = stereo_image->timestamp;

	carmen_velodyne_publish_variable_scan_message(&velodyne_partial_scan, lidar_num);
	

	cv::imshow("Bumblebee Image", open_cv_image) ;
	cv::imshow(neural_network, imgdepth * 250.0);
	waitKey(1);
}

void image_handler(camera_message *msg)
{
	camera_image *stereo_image = msg->images;
	Mat open_cv_image = Mat(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_data, 0); 
	//remap(open_cv_image, open_cv_image, MapX, MapY, INTER_LINEAR);											// Transforms the image to compensate for lens distortion

	cv::Mat imggray;
	cv::cvtColor(open_cv_image, imggray, cv::COLOR_BGR2GRAY);
	unsigned char *image_gray = imggray.data;

	cv::cvtColor(open_cv_image, open_cv_image, cv::COLOR_BGR2RGB);
	

	if (!strcmp(neural_network, "adabins"))
		depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "dpt"))
		depth_pred = libdpt_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "glpdepth"))
		depth_pred = libglpdepth_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "newcrfs"))
		depth_pred = libnewcrfs_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);

	cv::Mat imgdepth = cv::Mat(open_cv_image.rows, open_cv_image.cols, CV_16U, depth_pred);
	unsigned char *image_color = open_cv_image.data;
	convert_depth_to_velodyne_beams(depth_pred, vertical_resolution,
									horizontal_resolution, scan,
									range_max, image_gray, image_color);

	velodyne_partial_scan.partial_scan = scan;
	velodyne_partial_scan.number_of_shots = horizontal_resolution;
	velodyne_partial_scan.host = carmen_get_host();
	velodyne_partial_scan.timestamp = msg->timestamp;
	
	carmen_velodyne_publish_variable_scan_message(&velodyne_partial_scan, lidar_num);
	
	cv::imshow("Camera Driver Image", open_cv_image);
	cv::imshow(neural_network, imgdepth * 250.0);
	waitKey(1);
}

///////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

carmen_velodyne_shot *
alloc_velodyne_shot_scan_vector(int horizontal_resolution_l, int vertical_resolution_l)
{
	int i;

	carmen_velodyne_shot *vector = (carmen_velodyne_shot *)malloc(horizontal_resolution_l * sizeof(carmen_velodyne_shot));
	carmen_test_alloc(vector);

	for (i = 0; i < horizontal_resolution_l; i++)
	{
		vector[i].distance = (unsigned int *)calloc(vertical_resolution_l, sizeof(unsigned int));
		carmen_test_alloc(vector[i].distance);
		vector[i].intensity = (unsigned short *)calloc(vertical_resolution_l, sizeof(unsigned short));
		carmen_test_alloc(vector[i].intensity);
		// vector[i].point_color = (carmen_vector_3D_t *)calloc(vertical_resolution_l, sizeof(carmen_vector_3D_t));
		// carmen_test_alloc(vector[i].point_color);
		vector[i].shot_size = vertical_resolution_l;
	}

	return (vector);
}

static void
init_stereo_velodyne()
{
	scan = alloc_velodyne_shot_scan_vector(horizontal_resolution, vertical_resolution);

	reference_image = cvCreateImage(cvSize(camera_width, camera_height), IPL_DEPTH_8U, 3);
	reference_image_gray = cvCreateImage(cvSize(camera_width, camera_height), IPL_DEPTH_8U, 1);
}
/* per module */
static char ***variables, ***values;
static int **expert;
static int *num_params;
static int **update_param_mask;
static int **num_param_mask;

void concatenate(std::string &s, const char *c)
{
	s.reserve(s.size() + strlen(c));
	s.append(c);
}

static void init_params_edit()
{
	int m, p;

	carmen_param_get_modules(&modules, &num_modules);
	variables = (char ***)calloc(num_modules, sizeof(void *));
	carmen_test_alloc(variables);
	values = (char ***)calloc(num_modules, sizeof(void *));
	carmen_test_alloc(values);
	expert = (int **)calloc(num_modules, sizeof(void *));
	carmen_test_alloc(expert);
	num_params = (int *)calloc(num_modules, sizeof(int));
	carmen_test_alloc(num_params);
	update_param_mask = (int **)calloc(num_modules, sizeof(int *));
	carmen_test_alloc(update_param_mask);
	num_param_mask = (int **)calloc(num_modules, sizeof(int *));
	carmen_test_alloc(num_param_mask);

	for (m = 0; m < num_modules; m++)
	{
		carmen_param_get_all(modules[m], variables + m, values + m, expert + m,
							 num_params + m);
		update_param_mask[m] = (int *)calloc(num_params[m], sizeof(int));
		carmen_test_alloc(update_param_mask[m]);
		num_param_mask[m] = (int *)calloc(num_params[m], sizeof(int));
		carmen_test_alloc(num_param_mask[m]);
	}
	char module[256];
	char variable[256];
	char variable_ray_order[256];
	char variable_shot_size[256];

	string vertical_angles;
	string ray_order;
	double v_angle = (vertical_camera_angle / 2.0) * (-1);
	double delta_v_angle = vertical_camera_angle / camera_height;

	sprintf(module, "%s%d", "lidar", lidar_num);
	sprintf(variable, "%s", "vertical_angles");
	sprintf(variable_ray_order, "%s", "ray_order");
	sprintf(variable_shot_size, "%s", "shot_size");

	char value_string[256];
	char value_ray_string[256];
	for (int i = 0; i < camera_height; i++)
	{ // 480
		sprintf(value_string, "%0.4f ", v_angle);
		concatenate(vertical_angles, value_string);
		sprintf(value_ray_string, "%d ", camera_height - i - 1);
		concatenate(ray_order, value_ray_string);
		v_angle += delta_v_angle;
	}
	char *return_value;
	int status = 0;
	int n = vertical_angles.length();
	char char_vertical_angles[n + 1];
	strcpy(char_vertical_angles, vertical_angles.c_str());

	n = ray_order.length();
	char char_ray_order[n + 1];
	strcpy(char_ray_order, ray_order.c_str());

	char char_shot_size[256];
	sprintf(char_shot_size, "%d", camera_height);

	for (m = 0; m < num_modules; m++)
	{
		if (!strcmp(modules[m], module))
		{
			carmen_param_set_module(modules[m]);
			for (p = 0; p < num_params[m]; p++)
			{
				if (!strcmp(variables[m][p], variable))
				{
					update_param_mask[m][p] = -2;
					if (carmen_param_set_variable(variables[m][p], char_vertical_angles,
												  &return_value) < 0)
						status = -1;
					else
					{
						status = 1;
						update_param_mask[m][p] = 0;
					}
					cout << module << "_" << variable << " " << char_vertical_angles << endl;
				}
				if (!strcmp(variables[m][p], variable_ray_order))
				{
					update_param_mask[m][p] = -2;
					if (carmen_param_set_variable(variables[m][p], char_ray_order,
												  &return_value) < 0)
						status = -1;
					else
					{
						status = 1;
						update_param_mask[m][p] = 0;
					}
					cout << module << "_" << variable_ray_order << " " << char_ray_order << endl;
				}
				if (!strcmp(variables[m][p], variable_shot_size))
				{
					update_param_mask[m][p] = -2;
					if (carmen_param_set_variable(variables[m][p], char_shot_size,
												  &return_value) < 0)
						status = -1;
					else
					{
						status = 1;
						update_param_mask[m][p] = 0;
					}
					cout << module << "_" << variable_shot_size << " " << char_shot_size << endl;
				}
			}
			break;
		}
	}

	if (status == 1)
		cout << "Saving parameters...done" << endl;
	else if (status == 0)
		cout << "Saving parameters...nothing to save" << endl;
	else
		cout << "Saving parameters...failed" << endl;

	for (m = 0; m < num_modules; m++)
	{
		for (p = 0; p < num_params[m]; p++)
		{
			free(variables[m][p]);
			free(values[m][p]);
		}
		free(modules[m]);
		free(variables[m]);
		free(values[m]);
		free(update_param_mask[m]);
		free(num_param_mask[m]);
	}
	free(modules);
	free(variables);
	free(values);
	free(num_params);
	free(update_param_mask);
	free(num_param_mask);
}

int read_parameters(int argc, char **argv)
{
	int num_items;

	char stereo_string[256];
	char camera_string[256];

	sprintf(stereo_string, "%s%d", "stereo", atoi(argv[1]));
	camera = atoi(argv[1]);
	if (camera == 3 || camera == 5)
	{
		sprintf(camera_string, "%s%d", "bumblebee_basic", atoi(argv[1]));
	}else{
		sprintf(camera_string, "%s%d", "intelbras", atoi(argv[1]));
	}

	carmen_param_t param_list[] = {
		{(char *)stereo_string, (char *)"vertical_resolution", CARMEN_PARAM_INT, &vertical_resolution, 1, NULL},
		{(char *)stereo_string, (char *)"horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 1, NULL},
		{(char *)stereo_string, (char *)"range_max", CARMEN_PARAM_DOUBLE, &range_max, 1, NULL},
		{(char *)stereo_string, (char *)"width", CARMEN_PARAM_INT, &camera_width, 1, NULL},
		{(char *)stereo_string, (char *)"height", CARMEN_PARAM_INT, &camera_height, 1, NULL},
		{(char *)stereo_string, (char *)"horizontal_camera_angle", CARMEN_PARAM_DOUBLE, &horizontal_camera_angle, 1, NULL},
		{(char *)stereo_string, (char *)"vertical_camera_angle", CARMEN_PARAM_DOUBLE, &vertical_camera_angle, 1, NULL},
		{(char *)stereo_string, (char *)"vertical_top_cut", CARMEN_PARAM_INT, &vertical_top_cut, 1, NULL},
		{(char *)stereo_string, (char *)"vertical_down_cut", CARMEN_PARAM_INT, &vertical_down_cut, 1, NULL},
		{(char *)stereo_string, (char *)"horizontal_start_angle", CARMEN_PARAM_DOUBLE, &horizontal_start_angle, 1, NULL},
		{(char *)stereo_string, (char *)"range_multiplier_factor", CARMEN_PARAM_DOUBLE, &range_multiplier_factor, 1, NULL},
		{(char *)stereo_string, (char *)"lidar", CARMEN_PARAM_INT, &lidar_num, 1, NULL},
		{(char *)camera_string, (char *)"fx", CARMEN_PARAM_DOUBLE, &fx, 0, NULL},
		{(char *)camera_string, (char *)"fy", CARMEN_PARAM_DOUBLE, &fy, 0, NULL},
		{(char *)camera_string, (char *)"cu", CARMEN_PARAM_DOUBLE, &cu, 0, NULL},
		{(char *)camera_string, (char *)"cv", CARMEN_PARAM_DOUBLE, &camera_cv, 0, NULL},
		// {(char *)camera_string, (char *)"k1", CARMEN_PARAM_DOUBLE, &k1, 0, NULL},
		// {(char *)camera_string, (char *)"k2", CARMEN_PARAM_DOUBLE, &k2, 0, NULL},
		// {(char *)camera_string, (char *)"k3", CARMEN_PARAM_DOUBLE, &k3, 0, NULL},
		// {(char *)camera_string, (char *)"p1", CARMEN_PARAM_DOUBLE, &p1, 0, NULL},
		// {(char *)camera_string, (char *)"p2", CARMEN_PARAM_DOUBLE, &p2, 0, NULL}
		};
	su = get_stereo_instance(camera,camera_width,camera_height);
	point_cloud_message.points = NULL;
	/*SU stereo*/
	// if ((camera_width > 0) && (camera_height > 0))
	// {
	// 	su.width = camera_width;
	// 	su.height = camera_height;
	// }
	// su.baseline = 0.0;
	// su.fx = fx * su.width;
	// su.fy = fy * su.height;
	// su.xc = cu * su.width;
	// su.yc = camera_cv * su.height;
	// su.vfov = 2 * atan(camera_height / (2 * su.fy));
	// su.hfov = 2 * atan(camera_width / (2 * su.fx));
	// su.stereo_stride_x = 1.0;
	// su.stereo_stride_y = 1.0;
	/*----*/

	if (vertical_resolution > vertical_resolution - 0)
		carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	carmen_param_t param_optional_list[] =
		{
			{(char *)"commandline", (char *)"neural_network", CARMEN_PARAM_STRING, &neural_network, 0, NULL},
		};
	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	// double fx_rect = fx * camera_width;
	// double fy_rect = fy * camera_height;
	// double cu_rect = cu * camera_width;
	// double cv_rect = camera_cv * camera_height;

	// cameraMatrix = (Mat_<double>(3, 3) << fx_rect, 0, cu_rect, 0, fy_rect, cv_rect, 0, 0, 1);
	// newcameramtx = (Mat_<double>(3, 3) << fx_rect, 0, cu_rect, 0, fy_rect, cv_rect, 0, 0, 1);
	// distCoeffs = (Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
	// R1 = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

	// initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, newcameramtx, Size(camera_width, camera_height), CV_16SC2, MapX, MapY);

	init_params_edit();
	return (0);
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	register_ipc_messages();
	if (argc - 1 != 3)
		carmen_die("%s: Wrong number of parameters. stereo requires 3 parameters and received %d parameter(s). \n"
				   "Usage:\n %s <camera_number>"
				   " -neural_network <n>    : neural_network option (adabins, dpt or glpdepth)\n",
				   argv[0], argc - 1, argv[0]);

	camera = atoi(argv[1]);
	read_parameters(argc, argv);
	

	if (!strcmp(neural_network, "adabins"))
		initialize_python_context_adabins();

	if (!strcmp(neural_network, "dpt"))
		initialize_python_context_dpt();

	if (!strcmp(neural_network, "glpdepth"))
	{
		initialize_python_context_glpdepth();
	}
	if (!strcmp(neural_network, "newcrfs"))
	{
		initialize_python_context_newcrfs();
	}
	init_stereo_velodyne();

	carmen_velodyne_define_messages();
	if (camera == 3 || camera == 5)
	{
		carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t)bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		camera_drivers_subscribe_message(camera, NULL, (carmen_handler_t)image_handler, CARMEN_SUBSCRIBE_LATEST);
	}

	carmen_ipc_dispatch();

	return 0;
}
