
#include <cstdio>
#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

#define BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH 640
#define BUMBLEBEE_BASIC_VIEW_NUM_COLORS 3

static int received_image = 0;

static carmen_bumblebee_basic_stereoimage_message last_message;

static int msg_fps = 0, msg_last_fps = 0; //message fps
static int disp_fps = 0, disp_last_fps = 0; //display fps

char window_name[32];


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


static void
shutdown_camera_view(int x)
{
    if (x == SIGINT)
    {
        carmen_ipc_disconnect();
        printf("Disconnected from robot.\n");
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

	static int first = 1;
    static Mat *resized_image_left = NULL;
    static Mat *resized_image_right = NULL;

    Mat concat, bgr;
    Mat src_image_left(Size(msg->width, msg->height), CV_8UC3, msg->raw_left);
    Mat src_image_right(Size(msg->width, msg->height), CV_8UC3, msg->raw_right);

    update_fps(msg);

	window_scale = (double) BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH / (double) msg->width;
	window_height = (int) (msg->height * window_scale);

	//printf("window_scale: %lf window_height: %d width: %d msg: %d %d\n", window_scale, window_height, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH,
			//msg->height, msg->width);

    if (first)
    {
    	resized_image_left = new Mat(Size(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH, window_height), CV_8UC3);
    	resized_image_right = new Mat(Size(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH, window_height), CV_8UC3);
    	first = 0;
    }

    //resizing the image
    resize(src_image_left, *resized_image_left, resized_image_left->size());
    resize(src_image_right, *resized_image_right, resized_image_right->size());

    sprintf(msg_fps_string, "MSG_FPS: %d", msg_last_fps);
    sprintf(disp_fps_string, "DISP_FPS: %d", disp_last_fps);

    putText(*resized_image_left, msg_fps_string, cvPoint(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0));
    putText(*resized_image_right, disp_fps_string, cvPoint(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.7, cvScalar(255, 255, 0));

    hconcat(*resized_image_left, *resized_image_right, concat);
    cvtColor(concat, bgr, CV_RGB2BGR);
    imshow(window_name, bgr);

    char c = waitKey(2);
    process_event(c);
}


int
main(int argc, char **argv)
{
    int camera = 0;

    if (argc != 2)
    {
        fprintf(stderr, "%s: Wrong number of parameters. stereo requires 1 parameter and received %d. \n Usage: %s <camera_number>", argv[0], argc - 1, argv[0]);
        exit(1);
    }

    camera = atoi(argv[1]);
    sprintf(window_name, "bb%d", camera);

    // Just to open an initial window
    for (int i = 0; i < 10; i++)
    {
		imshow(window_name, Mat::zeros(Size(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH * 2, 480), CV_8UC3));
		waitKey(2);
    }

    carmen_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_camera_view);

    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_ipc_dispatch();

    return 0;
}

