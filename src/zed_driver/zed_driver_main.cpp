#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

using namespace std;
using namespace sl;
using namespace cv;


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
// Handlers																					 //
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
// Initializations																		     //
///////////////////////////////////////////////////////////////////////////////////////////////


int
read_parameters(int argc, char **argv, int &camera_id, int &fps)
{
	char camera_name[64];
    camera_id = atoi(argv[1]);
    int width, height;

	sprintf(camera_name, "ZED%d", camera_id);

	carmen_param_t param_list[] =
	{
		{camera_name, (char*) "height", CARMEN_PARAM_INT, &height, 0, NULL},
		{camera_name, (char*) "width",  CARMEN_PARAM_INT, &width, 0, NULL},
        {camera_name, (char*) "fps",    CARMEN_PARAM_INT, &fps, 0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));

	return 0;
}


void
set_ZED_stream(Camera &zed)
{
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::VGA; //2208*1242 => HD2K: 0; 1920*1080 => HD1080: 1; 1280*720 => HD720: 2, 672*376 => VGA: 3
    init_parameters.camera_fps = 15;                        // Set fps at 30

    ERROR_CODE state = zed.open(init_parameters);
    if (state != ERROR_CODE::SUCCESS)
	{
        cout << "Error " << state << ", exit program." << endl;
        exit(0);
    }

    cout << "\nVideo Stream Sucessfully Opened!!!\n" << endl;
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
    int camera_id, fps;

    check_parameters(argc, argv);
    
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

    read_parameters(argc, argv, camera_id, fps);

    carmen_bumblebee_basic_define_messages(camera_id);

    setup_bumblebee_basic_message(msg, 672, 376);

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
