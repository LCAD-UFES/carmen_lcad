#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

using namespace std;
using namespace sl;
using namespace cv;

int camera_id;
int zed_camera_sensor_fps;
int param_width;
int param_height;


void
check_parameters(int argc, char **argv)
{
	if (argc != 2)
	{
		carmen_die("---------------------------------------------\n Wrong number of parameters! \n---------------------------------------------\n\nUSAGE: %s <camera_id>\n\n", argv[0]);
		exit (0);
	}
}



///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Handlers																					 //
//																							 //
///////////////////////////////////////////////////////////////////////////////////////////////


void
shutdown_module(int signo)
{
    if (signo == SIGINT)
	{
        carmen_ipc_disconnect();
        printf("Signal %d received, exiting program ...\n", signo);
        exit(0);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Initializations																		     //
//																							 //
///////////////////////////////////////////////////////////////////////////////////////////////


static int
read_parameters(int argc, char **argv)
{
	int num_items;
    camera_id = atoi(argv[1]);
	char bb_name[1024];

	sprintf(bb_name, "bumblebee_basic%d", camera_id);

	carmen_param_t param_list[] =
	{
		{bb_name, (char *) "zed_fps", CARMEN_PARAM_INT, &zed_camera_sensor_fps, 0, NULL},
		{bb_name, (char *) "height", CARMEN_PARAM_INT, &param_height, 0, NULL},
		{bb_name, (char *) "width", CARMEN_PARAM_INT, &param_width, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


sl::RESOLUTION
define_zed_resolution()
{
	//2208*1242 => HD2K: 0; 1920*1080 => HD1080: 1; 1280*720 => HD720: 2, 672*376 => VGA: 3
	sl::RESOLUTION zed_camera_sensor_quality;

	int param_size = param_width + param_height;
	switch (param_size)
	{
	case (2208 + 1242):
		zed_camera_sensor_quality = RESOLUTION::HD2K;
		break;
	case (1920 + 1080):
		zed_camera_sensor_quality = RESOLUTION::HD1080;
		break;
	case (1280 + 720):
		zed_camera_sensor_quality = RESOLUTION::HD720;
		break;
	case (672 + 376):
		zed_camera_sensor_quality = RESOLUTION::VGA;
		break;
	default:
		std::cout << "Error: invalid resolution : " << std::endl;
		exit(1);
	}

	return (zed_camera_sensor_quality);
}


void
set_ZED_stream(Camera &zed)
{
    InitParameters init_parameters;
    init_parameters.camera_resolution = define_zed_resolution(); //2208*1242 => HD2K: 0; 1920*1080 => HD1080: 1; 1280*720 => HD720: 2, 672*376 => VGA: 3
    init_parameters.camera_fps = zed_camera_sensor_fps;                        // Set fps at 30

    ERROR_CODE state = zed.open(init_parameters);
    if (state != ERROR_CODE::SUCCESS)
	{
        cout << "Error " << state << ", exit program." << endl;
        exit(0);
    }

    sl::CalibrationParameters calibration_params = zed.getCameraInformation().camera_configuration.calibration_parameters;;

    cout << fixed;
    cout << setprecision(10);
    cout << "video stream sucessfully opened!!!\n" << endl;
    cout << init_parameters.camera_resolution << endl;

    cout << "left" << "\t\t\t" << "right" << endl;
	cout << "fx: " << calibration_params.left_cam.fx << "\t" << calibration_params.right_cam.fx << endl;
	cout << "fy: " << calibration_params.left_cam.fy << "\t" << calibration_params.right_cam.fy << endl;
	cout << "cx: " << calibration_params.left_cam.cx << "\t" << calibration_params.right_cam.cx << endl;
	cout << "cy: " << calibration_params.left_cam.cy << "\t" << calibration_params.right_cam.cy << endl;
	cout << "k1: " << calibration_params.left_cam.disto[0] << "\t" << calibration_params.right_cam.disto[0] << endl;
	cout << "k2: " << calibration_params.left_cam.disto[1] << "\t" << calibration_params.right_cam.disto[1] << endl;
	cout << "k3: " << calibration_params.left_cam.disto[2] << "\t" << calibration_params.right_cam.disto[2] << endl;
	cout << "p1: " << calibration_params.left_cam.disto[3] << "\t" << calibration_params.right_cam.disto[3] << endl;
	cout << "p2: " << calibration_params.left_cam.disto[4] << "\t" << calibration_params.right_cam.disto[4] << endl;
}


void
setup_bumblebee_basic_message(carmen_bumblebee_basic_stereoimage_message &msg, int width, int height)
{
    msg.host = carmen_get_host();
    msg.image_size = width * height * 3; // ZED RGB Images have 3 channels
    msg.width = width;
    msg.height = height;
    msg.isRectified = 1;
}


int 
main(int argc, char **argv)
{
    Camera zed;
    ERROR_CODE state;
    sl::Mat left_image, right_image;
    cv::Mat cv_left_image, cv_right_image;
    carmen_bumblebee_basic_stereoimage_message msg;

    check_parameters(argc, argv);
    
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

    read_parameters(argc, argv);

    carmen_bumblebee_basic_define_messages(camera_id);

    setup_bumblebee_basic_message(msg, param_width, param_height);

    set_ZED_stream(zed);

    while (1)
	{
        state = zed.grab();
	    msg.timestamp = carmen_get_time();

        if (state == ERROR_CODE::SUCCESS)           // A new image is available if grab() returns ERROR_CODE::SUCCESS
        {
            zed.retrieveImage(left_image,  VIEW::LEFT);
            zed.retrieveImage(right_image, VIEW::RIGHT);

            cv::Mat cv_left_image  = cv::Mat((int) left_image.getHeight(), (int) left_image.getWidth(), CV_8UC4, left_image.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::Mat cv_right_image = cv::Mat((int) right_image.getHeight(), (int) right_image.getWidth(), CV_8UC4, right_image.getPtr<sl::uchar1>(sl::MEM::CPU));

            cvtColor(cv_left_image,  cv_left_image,  CV_BGRA2RGB);
            cvtColor(cv_right_image, cv_right_image, CV_BGRA2RGB);

            msg.raw_left  = cv_left_image.data;
            msg.raw_right = cv_right_image.data;

            carmen_bumblebee_basic_publish_message(camera_id, &msg);

			// DO NOT ERASE, usefull for debug
            // cout <<"Image resolution: " << left_image.getWidth() << "-" << left_image.getHeight() << "    " << cv_left_image.rows << "x"  << cv_left_image. cols << " || Image timestamp: " << left_image.timestamp.data_ns << endl;
			// resize(cv_left_image, cv_left_image, Size(640, 480));     imshow("ZED Image", cv_left_image);      waitKey(1);
        }
    }

    zed.close();

    return (EXIT_SUCCESS);
}
