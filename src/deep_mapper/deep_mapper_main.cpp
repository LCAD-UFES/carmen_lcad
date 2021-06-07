
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
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

static int camera;
static int bumblebee_basic_width;
static int bumblebee_basic_height;
static PyObject *deepMapper, *inferencia, *initialize_function;


void
infer_depth(double width, double height,
		int dx, int dy, int w, int h,
		unsigned char *image_raw, double timestamp)
{

	npy_intp dims[3] = {height, width, 3};
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image_raw);
	// chama o método python que retorna o mapa de profundidade
	PyObject* python_result_array = (PyObject*) PyObject_CallFunction(inferencia, (char *) "(O)", numpyArray);
    
	if (PyErr_Occurred())
	        PyErr_Print();
    
	uchar *result_array = (uchar*)PyByteArray_AsString(python_result_array);
    
	if (PyErr_Occurred())
        PyErr_Print();
    
    cv::Mat depth_image(480, 640, CV_16UC1, result_array);
	cv::namedWindow("detph map", cv::WINDOW_NORMAL);
	cv::imshow("depth map", depth_image);
	cv::waitKey(1);

	Py_DECREF(result_array);
	Py_DECREF(python_result_array);
	Py_DECREF(numpyArray);
	
	// return (depth_image);
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
		err = IPC_publishData (CARMEN_GPS_GPHDT_MESSAGE_NAME, carmen_extern_gphdt_ptr);
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
bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	
	infer_depth(stereo_image->width, stereo_image->height,
			0, 0, 640, 380,
			stereo_image->raw_right, stereo_image->timestamp);
	// publicar o mapa de profundidade
}


void
camera_drivers_message_handler(camera_message *msg)
{
	infer_depth(msg->images[0].width, msg->images[0].height,
			0, 50, 640, 380,
			(unsigned char *) msg->images[0].raw_data, msg->timestamp);
	// publicar o mapa de profundidade
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		Py_XDECREF(inferencia);
    	Py_DECREF(deepMapper);
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
		{(char *) "commandline", (char *) "camera_id", CARMEN_PARAM_INT, &camera, 0, NULL},
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


int
initialize( )
{
	Py_Initialize();
    import_array();

    deepMapper = PyImport_ImportModule("deep_mapper");

    if (deepMapper == nullptr)
    {
        PyErr_Print();
        printf("error: fail to import module\n");
        return 0;
    }

    initialize_function = PyObject_GetAttrString(deepMapper, (char *)"initialize");
    if (initialize_function == nullptr)
    {
        PyErr_Print();
        printf("error: fail to get dictionary\n");
        return 0;
    }


    if (initialize_function && PyCallable_Check(initialize_function))
    {
        PyObject_CallObject(initialize_function, nullptr);
        inferencia = PyObject_GetAttrString(deepMapper, (char *)"inferenceDepth");
        if (inferencia == NULL || !PyCallable_Check(inferencia))
        {
            Py_Finalize();
            exit(printf("Error: Could not load the inferenceDepth.\n"));
        }

        if (PyErr_Occurred())
            PyErr_Print();
    }
    else
    {
        if (PyErr_Occurred())
            PyErr_Print();
        fprintf(stderr, "Cannot initiate object \n");
    }
	printf("DeepMapper: Loading succefully.\n");
	return 1;
}


void
subscribe_messages()
{
	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t)bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);
    camera_drivers_subscribe_message(camera, NULL, (carmen_handler_t) camera_drivers_message_handler, CARMEN_SUBSCRIBE_LATEST);
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char *argv[])
{
	if (argc != 3)
	{
		printf(" Usage: ./deep_mapper -camera_id 3\n");
		exit (1);
	}

	signal(SIGINT, shutdown_module);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	if(initialize()==0){
		printf("Errors detected!.");
		return 0;
	}

	read_parameters(argc, argv);

	subscribe_messages();
	carmen_ipc_dispatch();

	return (0);
}
