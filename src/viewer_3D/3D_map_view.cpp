
#include <cstdio>
#include <carmen/carmen.h>
#include <carmen/viewer_3D_interface.h>
#include <carmen/user_preferences.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>

using namespace std;
using namespace cv;

static int received_image = 0;

static carmen_viewer_3D_map_view_message last_message;

static int msg_fps = 0, msg_last_fps = 0; //message fps
static int disp_fps = 0, disp_last_fps = 0; //display fps

char window_name[32];

static int verbose = 0;

char *user_pref_filename = NULL;
const char *user_pref_module;
user_param_t *user_pref_param_list;
int user_pref_num_items;
int user_pref_window_width  = 640;
int user_pref_window_height = 480;
int user_pref_window_x = -1;
int user_pref_window_y = -1;
Size window_size;


void
save_image(const unsigned char *image, int w, int h, const char *file_name)
{
    FILE *image_file = (FILE *) NULL;
    int i, r, g, b;

    if (!file_name || !image)
        return;

    if ((image_file = fopen(file_name, "w")) == (FILE *) NULL)
        return;

    // Write the image format and comment
    fprintf(image_file, "P3\n# CREATOR: bumblebee save_image ()\n");

    // Write the image dimensions and range
    fprintf(image_file, "%d %d\n%d\n", w, h, 255);

    for (i = 0; i < h * w; i++)
    {
        r = image[3 * i + 0];
        g = image[3 * i + 1];
        b = image[3 * i + 2];
        fprintf(image_file, "%d\n%d\n%d\n", r, g, b);
    }

    // Closes image file
    fclose(image_file);
}


void
compose_output_path(char *dirname, char *filename, char **composed_path)
{
    *composed_path = (char *) malloc((strlen(dirname) + strlen(filename) + 2 /* 1 for '\0' e 1 for the '/' between <dirname>/<filename> */) * sizeof (char));
    sprintf((*composed_path), "%s/%s", dirname, filename);
}


void
compose_filename_from_timestamp(double timestamp, char **filename, char *extension)
{

    *filename = (char*) malloc(256 * sizeof (char));
    sprintf((*filename), "%.25f.%s", timestamp, extension);
}


void
create_stereo_filename_from_timestamp(double timestamp, char **img_filename)
{
    std::string ppm = "ppm";

    compose_filename_from_timestamp(timestamp, img_filename, (char*) ppm.c_str());
}


static void
save_images()
{
    std::string aux = "./";

    char *filename;
    char *filepath;

    create_stereo_filename_from_timestamp(last_message.timestamp, &filename);

    compose_output_path((char*) aux.c_str(), filename, &filepath);

    save_image(last_message.raw_image, last_message.width, last_message.height, filepath);

    printf("Saved file '%s'\n", filepath);
}


void
read_user_preferences(int argc, char** argv)
{
	static user_param_t param_list[] =
	{
		{"window_width",  USER_PARAM_TYPE_INT, &user_pref_window_width},
		{"window_height", USER_PARAM_TYPE_INT, &user_pref_window_height},
		{"window_x",      USER_PARAM_TYPE_INT, &user_pref_window_x},
		{"window_y",      USER_PARAM_TYPE_INT, &user_pref_window_y},
	};
	user_pref_module = basename(argv[0]);
	user_pref_param_list = param_list;
	user_pref_num_items = sizeof(param_list) / sizeof(param_list[0]);
	user_preferences_read(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
	user_preferences_read_commandline(argc, argv, user_pref_param_list, user_pref_num_items);

	window_size = Size(user_pref_window_width, user_pref_window_height);
}


void
set_user_preferences()
{
	if (user_pref_window_width > 0 && user_pref_window_height > 0)
		resizeWindow(window_name, window_size.width, window_size.height);
	if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
		moveWindow(window_name, user_pref_window_x, user_pref_window_y);
}


void
save_user_preferences()
{
	// Function cv::getWindowImageRect requires OpenCV version 3.4.1 or higher
#if	(CV_VERSION_MAJOR * 10000 + CV_VERSION_MINOR * 100 + CV_VERSION_REVISION) >= 30401
	Rect display = getWindowImageRect(window_name);
	user_pref_window_width  = display.width / (show_left + show_right);
	user_pref_window_height = display.height;
	user_pref_window_x = display.x;
	user_pref_window_y = display.y - 56;
	user_preferences_save(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
#endif
}


static void
shutdown(int sig)
{
    if (sig == SIGINT)
    {
        carmen_ipc_disconnect();
        save_user_preferences();
        exit(0);
    }
}


void
process_event(char c)
{
	if (c == 'c' || c == 'q')
		shutdown(SIGINT);

	if (c == 's')
		save_images();
}


void
update_fps(carmen_viewer_3D_map_view_message* image_msg)
{
    static double last_timestamp = 0.0;
    static double last_time = 0.0;
    double time_now = carmen_get_time();

    if (!received_image)
    {
        received_image = 1;
        last_timestamp = image_msg->timestamp;
        last_time = time_now;
    }

    if ((image_msg->timestamp - last_timestamp) > 1.0)
    {
        msg_last_fps = msg_fps;
        msg_fps = 0;
        last_timestamp = image_msg->timestamp;
    }

    msg_fps++;

    if ((time_now - last_time) > 1.0)
    {
        disp_last_fps = disp_fps;
        disp_fps = 0;
        last_time = time_now;
    }

    disp_fps++;

    last_message = *image_msg;
}

#define DEG(x)	((x) * 180.0 / M_PI)

static void
image_handler(carmen_viewer_3D_map_view_message *msg)
{
	static int msg_count = 0;
	msg_count++;
	if (verbose)
		printf("\n[3D_map_view]: Receiving carmen_viewer_3D_map_view_message # %04d\n"
				"               camera_pose    x=%7.2lf  y=%7.2lf  z=%7.2lf  roll=%7.2lf  pitch=%7.2lf  yaw=%7.2lf\n"
				"               camera_offset  x=%7.2lf  y=%7.2lf  z=%7.2lf  roll=%7.2lf  pitch=%7.2lf  yaw=%7.2lf\n", msg_count,
				msg->camera_pose.position.x, msg->camera_pose.position.y, msg->camera_pose.position.z,
				DEG(msg->camera_pose.orientation.roll), DEG(msg->camera_pose.orientation.pitch), DEG(msg->camera_pose.orientation.yaw),
				msg->camera_offset.position.x, msg->camera_offset.position.y, msg->camera_offset.position.z,
				DEG(msg->camera_offset.orientation.roll), DEG(msg->camera_offset.orientation.pitch), DEG(msg->camera_offset.orientation.yaw));

    double window_scale;
    int window_height;
    static char msg_fps_string[256];
    static char disp_fps_string[256];
    static char img_resol_string[256];

	static int first = 1;
    static Mat *resized_image = NULL;
    Mat src_image, bgr;

    update_fps(msg);

    if (first)
    {
    	destroyWindow(window_name);
    	window_scale = (double) user_pref_window_width / (double) msg->width;
    	window_height = (int) (msg->height * window_scale);
    	resized_image = new Mat(Size(user_pref_window_width, window_height), CV_8UC3);
    	first = 0;
    }

    src_image = Mat(Size(msg->width, msg->height), CV_8UC3, msg->raw_image);
    resize(src_image, *resized_image, resized_image->size());

    sprintf(msg_fps_string, "MSG_FPS: %d", msg_last_fps);
    sprintf(disp_fps_string, "DISP_FPS: %d", disp_last_fps);
    sprintf(img_resol_string, "%dx%d", msg->width, msg->height);

    putText(*resized_image, msg_fps_string, cvPoint(7, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0));
    putText(*resized_image, disp_fps_string, cvPoint(7, 42), CV_FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(255, 255, 0));
    putText(*resized_image, img_resol_string, cvPoint(7, 64), CV_FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(255, 255, 0));

    cvtColor(*resized_image, bgr, CV_RGB2BGR);
    imshow(window_name, bgr);

    char c = waitKey(2);
    process_event(c);
}


void
read_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "commandline", (char *) "verbose", CARMEN_PARAM_ONOFF, &verbose, 0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown);
	read_parameters(argc, argv);

	sprintf(window_name, "3D Map View");
	namedWindow(window_name);
	read_user_preferences(argc, argv);
	set_user_preferences();

	// Just to open an initial window
	for (int i = 0; i < 10; i++)
	{
		imshow(window_name, Mat::zeros(window_size, CV_8UC3));
		waitKey(2);
	}

	carmen_viewer_3D_subscribe_map_view_message(NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ipc_dispatch();
	return 0;
}
