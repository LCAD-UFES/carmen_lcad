
#include <cstdio>
#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/user_preferences.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>

using namespace std;
using namespace cv;

static int camera = 0;
static int show_left = 1;
static int show_right = 1;

static int received_image = 0;

static carmen_bumblebee_basic_stereoimage_message last_message;

static int msg_fps = 0, msg_last_fps = 0; //message fps
static int disp_fps = 0, disp_last_fps = 0; //display fps

char window_name[32];

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
create_stereo_filename_from_timestamp(double timestamp, char **left_img_filename, char **right_img_filename)
{
    std::string l_ppm = "l.ppm";
    std::string r_ppm = "r.ppm";

    compose_filename_from_timestamp(timestamp, left_img_filename, (char*) l_ppm.c_str());
    compose_filename_from_timestamp(timestamp, right_img_filename, (char*) r_ppm.c_str());
}


static void
save_camera_images()
{
    std::string aux = "./";

    char *left_filename, *right_filename;
    char *left_filepath, *right_filepath;

    create_stereo_filename_from_timestamp(last_message.timestamp, &left_filename, &right_filename);

    compose_output_path((char*) aux.c_str(), left_filename, &left_filepath);
    compose_output_path((char*) aux.c_str(), right_filename, &right_filepath);

    save_image(last_message.raw_left, last_message.width, last_message.height, left_filepath);
    save_image(last_message.raw_right, last_message.width, last_message.height, right_filepath);

    printf("Saved file '%s'\n", left_filepath);
}


void
read_preferences(int argc, char** argv)
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
	user_preferences_read(user_pref_module, user_pref_param_list, user_pref_num_items);
	user_preferences_read_commandline(argc, argv, user_pref_param_list, user_pref_num_items);

	window_size = Size(user_pref_window_width * (show_left + show_right), user_pref_window_height);

	if (user_pref_window_width >= 0 && user_pref_window_height >= 0)
		resizeWindow(window_name, window_size.width, window_size.height);
	if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
		moveWindow(window_name, user_pref_window_x, user_pref_window_y);
}


void
save_preferences()
{
	// Function cv::getWindowImageRect requires OpenCV version 3.4.1 or higher
#if	(CV_VERSION_MAJOR * 10000 + CV_VERSION_MINOR * 100 + CV_VERSION_REVISION) >= 30401
	Rect display = getWindowImageRect(window_name);
	user_pref_window_width  = display.width / (show_left + show_right);
	user_pref_window_height = display.height;
	user_pref_window_x = display.x;
	user_pref_window_y = display.y - 56;
	user_preferences_save(user_pref_module, user_pref_param_list, user_pref_num_items);
#endif
}


static void
shutdown_camera_view(int x)
{
    if (x == SIGINT)
    {
        carmen_ipc_disconnect();
        printf("Disconnected from robot.\n");
        save_preferences();
        exit(0);
    }
}


void
process_event(char c)
{
	if (c == 'c' || c == 'q')
		shutdown_camera_view(SIGINT);

	if (c == 's')
		save_camera_images();
}


void
update_fps(carmen_bumblebee_basic_stereoimage_message* image_msg)
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


static void
image_handler(carmen_bumblebee_basic_stereoimage_message* msg)
{
    double window_scale;
    int window_height;
    static char msg_fps_string[256];
    static char disp_fps_string[256];
    static char img_resol_string[256];

	static int first = 1;
    static Mat *resized_image_left = NULL;
    static Mat *resized_image_right = NULL;
    Mat src_image_left, src_image_right, *text_image, concat, bgr;

    update_fps(msg);

    if (first)
    {
    	window_scale = (double) user_pref_window_width / (double) msg->width;
    	window_height = (int) (msg->height * window_scale);
    	resized_image_left = new Mat(Size(user_pref_window_width, window_height), CV_8UC3);
    	resized_image_right = new Mat(Size(user_pref_window_width, window_height), CV_8UC3);
    	first = 0;
    }

    if (show_left)
    {
    	src_image_left = Mat(Size(msg->width, msg->height), CV_8UC3, msg->raw_left);
    	resize(src_image_left, *resized_image_left, resized_image_left->size());
    }
    if (show_right)
    {
    	src_image_right = Mat(Size(msg->width, msg->height), CV_8UC3, msg->raw_right);
    	resize(src_image_right, *resized_image_right, resized_image_right->size());
    }

    sprintf(msg_fps_string, "MSG_FPS: %d", msg_last_fps);
    sprintf(disp_fps_string, "DISP_FPS: %d", disp_last_fps);
    sprintf(img_resol_string, "%dx%d", msg->width, msg->height);

    text_image = show_left ? resized_image_left : resized_image_right;
    putText(*text_image, msg_fps_string, cvPoint(7, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0));
    putText(*text_image, disp_fps_string, cvPoint(7, 42), CV_FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(255, 255, 0));
    putText(*text_image, img_resol_string, cvPoint(7, 64), CV_FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(255, 255, 0));

    if (show_left && show_right)
    	hconcat(*resized_image_left, *resized_image_right, concat);
    else
    	concat = show_left ? *resized_image_left : *resized_image_right;

    cvtColor(concat, bgr, CV_RGB2BGR);
    imshow(window_name, bgr);

    char c = waitKey(2);
    process_event(c);
}


void
usage(char *prog_name)
{
    fprintf(stderr, "\nUsage: %s  <camera_number>  -show <L|R|S|left|right|stereo>\n\n", prog_name);
    exit(1);
}


void
read_parameters(int argc, char **argv, int *camera, int *show_left, int *show_right)
{
	char *endptr;

	*camera = strtol(argv[1], &endptr, 0);
	if ((*endptr != '\0') || (*camera <= 0))
	{
		fprintf(stderr, "\nInvalid camera number: %s\n", argv[1]);
		usage(argv[0]);
	}

	if (argc >= 3 && strcmp(argv[2], "-show") == 0)
	{
		if (argc > 3)
		{
			if (strcmp(argv[3], "L") == 0 || strcmp(argv[3], "l") == 0 || strcmp(argv[3], "left") == 0)
				*show_left = 1, *show_right = 0;
			else if (strcmp(argv[3], "R") == 0 || strcmp(argv[3], "r") == 0 || strcmp(argv[3], "right") == 0)
				*show_left = 0, *show_right = 1;
			else if (strcmp(argv[3], "S") == 0 || strcmp(argv[3], "s") == 0 || strcmp(argv[3], "stereo") == 0)
				*show_left = 1, *show_right = 1;
			else
			{
				fprintf(stderr, "\nInvalid show option: %s\n", argv[3]);
				usage(argv[0]);
			}
		}
		else
		{
			fprintf(stderr, "\nMissing option after: %s\n", argv[2]);
			usage(argv[0]);
		}
	}
}


int
main(int argc, char **argv)
{
	if (argc == 1 || strcmp(argv[1], "-h") == 0)
		usage(argv[0]);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_camera_view);

	read_parameters(argc, argv, &camera, &show_left, &show_right);

	sprintf(window_name, "bb%d", camera);
	namedWindow(window_name);
	read_preferences(argc, argv);

	// Just to open an initial window
	for (int i = 0; i < 10; i++)
	{
		imshow(window_name, Mat::zeros(window_size, CV_8UC3));
		waitKey(2);
	}

	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ipc_dispatch();
	return 0;
}
