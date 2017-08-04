#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/stereo_interface.h>
#include "zed_camera_sensor_interface.h"

//ZED Includes
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

//opencv includes
#include <opencv2/opencv.hpp>

#define BUMBLEBEE_ID 4
//static int zed_camera_sensor_getkey = 0;
sl::zed::Camera* zed;

//carmen_zed_camera_sensor_stereoimage_message stereo_msg;
carmen_zed_camera_sensor_depthmap_message    depth_msg;

int zed_camera_sensor_quality = 0;
//#HD2K: 0, HD1080: 1, HD720: 2, VGA: 3
double zed_camera_sensor_fps = 0.0;
/*
 *RESOLUTION_HD2K 2208*1242, available framerates: 15 fps.
 *RESOLUTION_HD1080 1920*1080, available framerates: 15, 30 fps.
 *RESOLUTION_HD720 1280*720, available framerates: 15, 30, 60 fps.
 *RESOLUTION_VGA 	672*376, available framerates: 15, 30, 60, 100 fps.
 */

int param_width = 0.0;
int param_height = 0.0;

//static int disp_fps = 0;
//static int disp_last_fps = 0; //display fps

using namespace std;
using namespace cv;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
carmen_bumblebee_publish_stereoimage_message(unsigned char *rawLeft, unsigned char *rawRight, int width, int height, int channels)
{
	static int first = 1;
	static Mat *frameLeft_3 = NULL;
    static Mat *frameRight_3 = NULL;

    Mat frameLeft_4(height, width, CV_8UC4, rawLeft, 4 * width);
    Mat frameRight_4(height, width, CV_8UC4, rawRight, 4 * width);

    if (first)
    {
    	frameLeft_3 = new Mat(Size(width, height), CV_8UC3);
    	frameRight_3 = new Mat(Size(width, height), CV_8UC3);
    	first = 0;
    }

    cvtColor(frameLeft_4, *frameLeft_3, CV_BGRA2RGB);
    cvtColor(frameRight_4, *frameRight_3, CV_BGRA2RGB);

	carmen_bumblebee_basic_stereoimage_message stereo_msg;

	stereo_msg.timestamp = carmen_get_time();
	stereo_msg.host = carmen_get_host();
	stereo_msg.image_size = width * height * channels;
	stereo_msg.width = width;
	stereo_msg.isRectified = 1;
	stereo_msg.height = height;
	stereo_msg.raw_left = frameLeft_3->data;
	stereo_msg.raw_right = frameRight_3->data;
//	printf("Publicando \n");
	carmen_bumblebee_basic_publish_message(BUMBLEBEE_ID, &stereo_msg);
}

void
carmen_zed_publish_depthmap_message(unsigned char *rawImage, int width, int height, int bytes_per_pixel)
{
	depth_msg.timestamp = carmen_get_time();
	depth_msg.host = carmen_get_host();
	depth_msg.image_size = width*height*bytes_per_pixel;
	depth_msg.width = width;
	depth_msg.height = height;
	depth_msg.raw_image = rawImage;

	carmen_zed_camera_sensor_publish_depthmap_message(&depth_msg);
}


void
carmen_stereo_publish_depthmap_message(sl::zed::Mat depth, unsigned char *reference_image, int width, int height, int channels)
{
	carmen_simple_stereo_disparity_message disparity_message;
	Mat frameLeft_4(height, width, CV_8UC4, reference_image, 4 * width);
	Mat frameLeft_3;
	cvtColor(frameLeft_4, frameLeft_3, CV_BGRA2RGB);

	disparity_message.host = carmen_get_host();
	disparity_message.timestamp = carmen_get_time();
	disparity_message.disparity = (float*) depth.data;
	disparity_message.disparity_size = width*height;
	disparity_message.reference_image = frameLeft_3.data;
	disparity_message.reference_image_size = width*height*channels;

	carmen_stereo_publish_message(BUMBLEBEE_ID, &disparity_message);
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		delete zed;
		carmen_ipc_disconnect();
		printf("zed_camera_sensor: disconnected.\n");
		exit(0);
	}
}

static int read_parameters(int argc, char **argv)
{
	int num_items;
	char bb_name[64];

	sprintf(bb_name, "bumblebee_basic%d", BUMBLEBEE_ID);

	carmen_param_t param_list[] =
	{
			{bb_name, (char*) "zed_fps", CARMEN_PARAM_DOUBLE, &zed_camera_sensor_fps, 0, NULL},
			{bb_name, (char*) "height", CARMEN_PARAM_INT, &param_height, 0, NULL},
			{bb_name, (char*) "width", CARMEN_PARAM_INT, &param_width, 0, NULL}

	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

void setupCamera() {
	std::cout << "RESOLUTION : " << zed_camera_sensor_quality << std::endl;
	zed = new sl::zed::Camera(static_cast<sl::zed::ZEDResolution_mode>(zed_camera_sensor_quality), zed_camera_sensor_fps,0);
	sl::zed::InitParams params;
	params.mode = sl::zed::MODE::NONE;
	sl::zed::ERRCODE err = zed->init(params);
	if (err != sl::zed::SUCCESS) {
		std::cout << "Error code : " << sl::zed::errcode2str(err) << std::endl;
		delete zed;
		exit(0);
	}

	// the depth is limited to 20. METERS as define in zed::init()
	// Set the maximum distance of depth/disparity estimation (all values after this limit will be reported as TOO_FAR value)
	zed->setDepthClampValue(5000);

//	sl::zed::ZED_SELF_CALIBRATION_STATUS old_self_calibration_status = sl::zed::SELF_CALIBRATION_NOT_CALLED;

	//Jetson only. Execute the calling thread on core 2
	sl::zed::Camera::sticktoCPUCore(2);
}


void
define_zed_resolution()
{
	//2208*1242 => HD2K: 0; 1920*1080 => HD1080: 1; 1280*720 => HD720: 2, 672*376 => VGA: 3
	int param_size = param_width+param_height;
	switch (param_size)
	{
	case (2208+1242):
					zed_camera_sensor_quality = 0;
	break;
	case (1920+1080):
					zed_camera_sensor_quality = 1;
	break;
	case (1280+720):
					zed_camera_sensor_quality = 2;
	break;
	case (672+376):
					zed_camera_sensor_quality = 3;
	break;
	default:
		std::cout << "Error: wrong resolution : " << std::endl;
		exit(1);
	}
}


int main(int argc, char **argv)
{

	if (argc != 2)
		carmen_die("%s: Wrong number of parameters. %s requires 1 parameter and received %d parameter(s). \nUsage:\n %s <stereo|depth>\n", argv[0], argv[0], argc-1, argv[0]);

	bool isStereo = (!strcmp(argv[1],"stereo") ? true : false );

	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Read application specific parameters (Optional) */
	read_parameters(argc, argv);
	//carmen_param_allow_unfound_variables(0);
	//ZED uses type enum RESOLUTION
	define_zed_resolution();

	/* Define published messages by your module */
	//carmen_zed_camera_sensor_define_depthmap_messages();
	//  carmen_zed_camera_sensor_define_stereoimage_messages();
	carmen_stereo_define_messages(BUMBLEBEE_ID);
	carmen_bumblebee_basic_define_messages(BUMBLEBEE_ID);

	/* Setting up the ZED Camera */
	setupCamera();
	/*  DELAY ATUAL = 21.46-20.98 =>
	 * 20.12 - 19.69
	 * 18.96-18.48
	 * 17.43 - 16.93
	 * */


	int width = zed->getImageSize().width;
	int height = zed->getImageSize().height;

	int ConfidenceIdx = 100;
	//STANDARD: Structure conservative, no occlusion filling. Application example : Obstacle detection, 3D reconstructions, 3D measures
	//FILL: Occlusion filling, edge sharpening, advanced post-filtering. Application example : Refocusing, Multi-view generation
	sl::zed::SENSING_MODE dm_type = sl::zed::STANDARD;

	if(isStereo)
	{
		while(1)
		{
			//---colar aqui PART1 calular FPS

			bool res = zed->grab(dm_type);

			if(!res)
			{
				//---colar aqui PART2 para imprimir FPS/mostrar imagem

//				zed->setConfidenceThreshold(ConfidenceIdx);
				carmen_bumblebee_publish_stereoimage_message(zed->retrieveImage(sl::zed::SIDE::LEFT).data, zed->retrieveImage(sl::zed::SIDE::RIGHT).data, width, height, 3);
//				ConfidenceIdx = ConfidenceIdx < 1 ? 1 : ConfidenceIdx;
//				ConfidenceIdx = ConfidenceIdx > 100 ? 100 : ConfidenceIdx;
			}
//			usleep(1000/zed->getCurrentFPS());
			res = zed->grab(dm_type);
		}
	}
	else
	{
		while(1)
		{
			bool res = zed->grab(dm_type);
			if(!res)
			{
				zed->setConfidenceThreshold(ConfidenceIdx);
				carmen_stereo_publish_depthmap_message((zed->retrieveMeasure(sl::zed::MEASURE::DISPARITY)), (zed->retrieveImage(sl::zed::SIDE::LEFT).data), width, height, 3);
				//              carmen_zed_publish_depthmap_message(zed->normalizeMeasure(sl::zed::MEASURE::DEPTH).data, width, height, 4);
				ConfidenceIdx = ConfidenceIdx < 1 ? 1 : ConfidenceIdx;
				ConfidenceIdx = ConfidenceIdx > 100 ? 100 : ConfidenceIdx;
			}
		}
	}
	//-----PART1----------------------------
	//			static double last_time = 0.0;
	//			double time_now = carmen_get_time();
	//			if ((time_now - last_time) > 1.0)
	//			{
	//				disp_last_fps = disp_fps;
	//				disp_fps = 0;
	//				last_time = time_now;
	//			}
	//-------------------------------------
	//PART2 --- *Codigo para Mostrar imagem sem publicar - teste do delay*/
	//        	  cv::Mat imagem(height, width, CV_8UC4);
	//        	  for (int i  = 0; i < 4;)

	//        	  sl::zed::slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::LEFT)).copyTo(imagem);
	//        	  cv::imshow("Janela", imagem);
	//        	  cv::waitKey(1);
	//			disp_fps++;
	//			printf("Atual FPS: %d \n", disp_last_fps);

}
