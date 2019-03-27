
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "velodyne_camera_calibration.h"

int bumblebee_received = 0;

int camera_number;

carmen_bumblebee_basic_stereoimage_message bumblebee_message;

static carmen_pose_3D_t camera_pose; // Camera pose in relation to sensor board
static carmen_pose_3D_t velodyne_pose; //velodyne pose in relation to sensor board

static carmen_camera_parameters camera_parameters;

void
process_key_input(char k)
{
    static double step_size = 0.01;
    static double angular_step_size = 0.5;

    // Nao foi utilizado um switch case por questao de legibilidade do codigo, dessa maneira ele fica mais compacto e
    // simples de ler
    if (k == 'q') camera_pose.position.x += step_size;
    if (k == 'a') camera_pose.position.x -= step_size;
    if (k == 'w') camera_pose.position.y += step_size;
    if (k == 's') camera_pose.position.y -= step_size;
    if (k == 'e') camera_pose.position.z += step_size;
    if (k == 'd') camera_pose.position.z -= step_size;

    if (k == 'r') camera_pose.orientation.roll += carmen_degrees_to_radians(angular_step_size);
    if (k == 'f') camera_pose.orientation.roll -= carmen_degrees_to_radians(angular_step_size);
    if (k == 't') camera_pose.orientation.pitch += carmen_degrees_to_radians(angular_step_size);
    if (k == 'g') camera_pose.orientation.pitch -= carmen_degrees_to_radians(angular_step_size);
    if (k == 'y') camera_pose.orientation.yaw += carmen_degrees_to_radians(angular_step_size);
    if (k == 'h') camera_pose.orientation.yaw -= carmen_degrees_to_radians(angular_step_size);

    if (k == 'u') camera_parameters.fx_factor += step_size;
    if (k == 'j') camera_parameters.fx_factor -= step_size;

    if (k == 'i') camera_parameters.fy_factor += step_size;
    if (k == 'k') camera_parameters.fy_factor -= step_size;

    if(k == 'z') step_size = step_size / 10.0;
    if(k == 'x') step_size = step_size * 10.0;

    if(k == 'n') angular_step_size = angular_step_size / 10.0;
    if(k == 'm') angular_step_size = angular_step_size * 10.0;


    if (k == 'q' || k == 'a' || k == 'w' || k == 's' || k == 'e' || k == 'd' ||
        k == 'r' || k == 'f' || k == 't' || k == 'g' || k == 'y' || k == 'h' ||
        k == 'u' || k == 'j' || k == 'i' || k == 'k')
    {
        printf("\nCAM POSE:\n\tx:%lf y:%lf z:%lf\n",
               camera_pose.position.x, camera_pose.position.y, camera_pose.position.z);
        printf("\troll: %lf pitch: %lf yaw: %lf\n",
               camera_pose.orientation.roll, camera_pose.orientation.pitch, camera_pose.orientation.yaw);
        printf("\tfx: %lf, fy:%lf\n", camera_parameters.fx_factor, camera_parameters.fy_factor);
    }

    if(k == 'z' || k == 'x')
        printf("\nValor do passo alterado para: %lf\n", step_size);

    if(k == 'm' || k == 'n')
        printf("\nValor do passo angular alterado para: %lf\n", angular_step_size);

}


void
show_velodyne(carmen_velodyne_partial_scan_message *velodyne_message)
{
    if (!bumblebee_received)
        return;

    cv::Mat camera_image(cv::Size(bumblebee_message.width, bumblebee_message.height), CV_8UC3, bumblebee_message.raw_right);
    cv::Mat camera_image_show(cv::Size(bumblebee_message.width, bumblebee_message.height), CV_8UC3);

    cv::cvtColor(camera_image,camera_image_show, CV_RGB2BGR);

    std::vector<carmen_velodyne_points_in_cam_t> points_in_cam = carmen_velodyne_camera_calibration_lasers_points_in_camera(velodyne_message, camera_parameters,
                                                                                                                            velodyne_pose, camera_pose,
                                                                                                                            bumblebee_message.width,
                                                                                                                            bumblebee_message.height);

    for(unsigned int i = 0; i < points_in_cam.size(); i++)
    {
        float r = 0;

        r = points_in_cam.at(i).laser_polar.length / 50.0;
        r *= 255;
        r = 255 - r;

        cv::circle(camera_image_show,cv::Point(points_in_cam.at(i).ipx,points_in_cam.at(i).ipy),2,cv::Scalar(0,0,r));
    }

    cv::imshow("Camera with points", camera_image_show);

    char k = cv::waitKey(5);
    process_key_input(k);

}


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
bumblebee_basic_image_handler(carmen_bumblebee_basic_stereoimage_message *bumblebee_basic_message __attribute__ ((unused)))
{
	bumblebee_received = 1;
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
    carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message);
	show_velodyne(velodyne_message);
}


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();
        fprintf(stderr, "\nShutdown velodyne_camera_calibration\n");

        exit(0);
    }
}


static void
subscribe_to_ipc_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera_number, &bumblebee_message,
                                                 (carmen_handler_t) bumblebee_basic_image_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL,
                                                   (carmen_handler_t)velodyne_partial_scan_message_handler,
                                                   CARMEN_SUBSCRIBE_LATEST);

}

int
read_parameters(int argc, char **argv)
{

    if ((argc != 2))
        carmen_die("%s: Wrong number of parameters. This module requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number>",
                   argv[0], argc - 1, argv[0]);

    /* defining the camera to be used */
    camera_number = atoi(argv[1]);

    int num_items;
	char bumblebee_string[256];
    char camera_string[256];

	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera_number);
    sprintf(camera_string, "%s%d", "camera", camera_number);

	carmen_param_t param_list[] = {

            { bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
            { bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
            { bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
            { bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
            { bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },

            {(char *) "velodyne",  (char *) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
            {(char *) "velodyne",  (char *) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
            {(char *) "velodyne",  (char *) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
            {(char *) "velodyne",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
            {(char *) "velodyne",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
            {(char *) "velodyne",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

            { camera_string, (char*) "x", CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
            { camera_string, (char*) "y", CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
            { camera_string, (char*) "z", CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
            { camera_string, (char*) "roll", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
            { camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
            { camera_string, (char*) "yaw", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL }

    };


	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////

int
main(int argc, char **argv)
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

    /* Initialize all the relevant parameters */
    read_parameters(argc, argv);

    /* Subscribe to relevant messages */
    subscribe_to_ipc_messages();

	carmen_ipc_dispatch();

	return 0;
}

