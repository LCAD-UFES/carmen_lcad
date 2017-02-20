/* Carmen includes */
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>

#include <carmen/carmen_graphics.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;


// Image show
static GtkWidget *drawing_area;
static GdkPixbuf *src_buffer;

// Parameters
static int image_width;
static int image_height;
static int window_view_height;
static int window_view_width;

// Camera Index
static int camera;

// Image vector
#define IMAGE_VECTOR_SIZE 10

typedef struct
{
	unsigned char *traffic_light_image;
	double timestamp;
} image_vector_t;

image_vector_t image_vector[IMAGE_VECTOR_SIZE];
int image_vector_index = 0;


static void
shutdown_traffic_light_view(int x)
{
    if (x == SIGINT)
    {
        carmen_verbose("Disconnecting Traffic Light View Service.\n");
        carmen_ipc_disconnect();

        exit(1);
    }
}


static void
redraw_viewer(void)
{
    if (src_buffer)
    {
        gdk_draw_pixbuf(drawing_area->window,
                        drawing_area->style->fg_gc[GTK_WIDGET_STATE(drawing_area)],
                        GDK_PIXBUF(src_buffer), 0, 0, 0, 0,
                        drawing_area->allocation.width,
                        drawing_area->allocation.height,
                        GDK_RGB_DITHER_NONE, 0, 0);
    }
}


static gint
expose_event(GtkWidget *widget __attribute__((unused)), GdkEventExpose *event __attribute__((unused)))
{
    return (1);
}


static gint
key_press_event(GtkWidget *widget __attribute__((unused)), GdkEventKey *key)
{
    if (toupper(key->keyval) == 'C')// && (key->state & GDK_CONTROL_MASK))
        shutdown_traffic_light_view(SIGINT);

    if (toupper(key->keyval) == 'Q')// && (key->state & GDK_CONTROL_MASK))
        shutdown_traffic_light_view(SIGINT);

    if (key->state || key->keyval > 255)
        return (1);

    return (1);
}


static gint
key_release_event(GtkWidget *widget __attribute__((unused)), GdkEventButton *key __attribute__((unused)))
{
    return (1);
}


void
add_traffic_light_information_to_image(cv::Mat &image, carmen_traffic_light_message *message)
{
	ostringstream myStream;

	myStream << fixed << setprecision(1) << message->distance << flush;
	string text;
	if (message->distance <= 200.0 && message->distance != -1.0)
		text = "Distance " + myStream.str() + " meters";
	else
		text = "Distance greater than 200 meters";

	if (image.cols > 900)
		putText(image, text, cv::Point(20, 900), FONT_HERSHEY_COMPLEX, 1, Scalar(100, 255, 100), 2, 8);
	else
		putText(image, text, cv::Point(20, 400), FONT_HERSHEY_COMPLEX, 1, Scalar(100, 255, 100), 2, 8);

	int num_red = 0;
	for (int i = 0; i < message->num_traffic_lights; i++)
	{
		if (message->traffic_lights[i].color == TRAFFIC_LIGHT_RED)
			num_red++;

		CvPoint p1, p2;
		p1.x = message->traffic_lights[i].x1;
		p1.y = message->traffic_lights[i].y1;
		p2.x = message->traffic_lights[i].x2;
		p2.y = message->traffic_lights[i].y2;
		if (message->traffic_lights[i].color == TRAFFIC_LIGHT_RED)
			cv::rectangle(image, p1, p2, CV_RGB(0, 0, 255), 3, 10, 0);
		else if (message->traffic_lights[i].color == TRAFFIC_LIGHT_GREEN)
			cv::rectangle(image, p1, p2, CV_RGB(0, 255, 0), 3, 10, 0);
		else // yellow
			cv::rectangle(image, p1, p2, CV_RGB(0, 255, 255), 3, 10, 0);
	}

	if (message->num_traffic_lights > 0)
	{
		if (num_red > 0) // @@@ Alberto: Isso eh pessimista e nao trata amarelo
			circle(image, Point(50, 130), 50, Scalar(255, 0, 0), -1, 8);
		else
			circle(image, Point(50, 130), 50, Scalar(0, 255, 0), -1, 8);
	}
}


unsigned char *
get_nearest_image(carmen_traffic_light_message *message)
{
	int nearest_image_index = 0;
	double smalest_difference = message->timestamp;

    for (int i = 0; i < IMAGE_VECTOR_SIZE; i++)
    {
    	double difference = fabs(image_vector[i].timestamp - message->timestamp);
		if (difference < smalest_difference)
		{
			nearest_image_index = i;
			smalest_difference = difference;
		}
    }

    if (image_vector[nearest_image_index].timestamp != 0.0)
    	return (image_vector[nearest_image_index].traffic_light_image);
    else
    	return (NULL);
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

// none...

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_traffic_light_message_handler(carmen_traffic_light_message *message)
{
    unsigned char *nearest_image = get_nearest_image(message);
    if (nearest_image == NULL)
    	return;

    cv::Mat image(image_height, image_width, CV_8UC3);
	memcpy(image.data, nearest_image, 3 * image_height * image_width);

	add_traffic_light_information_to_image(image, message);

	cv::Mat resized_image;
	resized_image.create(window_view_height, window_view_width, CV_8UC3);
	cv::Size size(window_view_width, window_view_height);

#if CV_MAJOR_VERSION == 2
	resize(image, resized_image, size);
#elif CV_MAJOR_VERSION == 3
	cv::resize(image, resized_image, size);
#endif
    src_buffer = gdk_pixbuf_new_from_data(resized_image.data, GDK_COLORSPACE_RGB,
         FALSE, 8, window_view_width, window_view_height, window_view_width * 3, NULL, NULL);

    redraw_viewer();
}


void
carmen_bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	memcpy(image_vector[image_vector_index].traffic_light_image, stereo_image->raw_right, 3 * image_height * image_width);
	image_vector[image_vector_index].timestamp = stereo_image->timestamp;

	image_vector_index++;
	if (image_vector_index >= IMAGE_VECTOR_SIZE)
		image_vector_index = 0;
}


static gint
updateIPC(gpointer *data __attribute__((unused)))
{
    carmen_ipc_sleep(0.01);
    carmen_graphics_update_ipc_callbacks((GdkInputFunction) updateIPC);

    return (1);
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
start_graphics(int argc, char *argv[])
{
    GtkWidget *main_window;

    gtk_init(&argc, &argv);

    main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    gtk_window_set_title(GTK_WINDOW(main_window), "Traffic Light View");
    drawing_area = gtk_drawing_area_new();

    gtk_widget_set_usize(drawing_area, window_view_width, window_view_height);

    //gtk_widget_set_usize(drawing_area, window_view_width + window_view_width + window_view_width + window_view_width + window_view_width + 40, window_view_height);

    gtk_container_add(GTK_CONTAINER(main_window), drawing_area);

    gtk_signal_connect(GTK_OBJECT(drawing_area), "expose_event", (GtkSignalFunc) expose_event, NULL);
    gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event", (GtkSignalFunc) key_press_event, NULL);
    gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event", (GtkSignalFunc) key_release_event, NULL);

    gtk_widget_add_events(drawing_area,
                          GDK_EXPOSURE_MASK | GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK
                          | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK
                          | GDK_KEY_PRESS_MASK | GDK_KEY_RELEASE_MASK);

    carmen_graphics_update_ipc_callbacks((GdkInputFunction) updateIPC);
    gtk_widget_show(drawing_area);
    gtk_widget_show(main_window);

    gtk_main();
}


static int
read_parameters(int argc, char **argv)
{
    int num_items;
    char bumblebee_string[256];

    if (argc < 2)
        carmen_die("%s: Wrong number of parameters. Traffic Light Viewer requires a parameter.\nUsage:\n %s <camera_number>\n",
                   argv[0], argv[0]);

    camera = atoi(argv[1]);

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

    carmen_param_t param_list[] =
    {
        { (char*) bumblebee_string,(char *) "width", CARMEN_PARAM_INT, &image_width, 0, NULL},
        { (char*) bumblebee_string,(char *) "height", CARMEN_PARAM_INT, &image_height, 0, NULL},
        { (char*) "traffic_light_viewer",(char *) "width", CARMEN_PARAM_INT, &window_view_width, 0, NULL},
        { (char*) "traffic_light_viewer",(char *) "height", CARMEN_PARAM_INT, &window_view_height, 0, NULL}
    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return (0);
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);
    signal(SIGINT, shutdown_traffic_light_view);
    read_parameters(argc, argv);

    carmen_traffic_light_subscribe(camera, NULL, (carmen_handler_t) carmen_traffic_light_message_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler,
    		CARMEN_SUBSCRIBE_LATEST);

    memset(image_vector, 0, IMAGE_VECTOR_SIZE * sizeof(image_vector_t));
    for (int i = 0; i < IMAGE_VECTOR_SIZE; i++)
		image_vector[i].traffic_light_image = (unsigned char *) malloc(3 * image_height * image_width * sizeof(unsigned char));

    start_graphics(argc, argv);

    return (0);
}
