/* Carmen includes */
#include <carmen/carmen_graphics.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>

#include <opencv2/legacy/legacy.hpp>
/* Image show */
static GtkWidget *drawing_area;
static GdkPixbuf *src_buffer;

static IplImage *right_image = NULL;

// Parameters //
static int image_width;
static int image_height;
static int window_view_height;
static int window_view_width;
/* Camera Index */
static int camera;
carmen_traffic_light_message traffic_light_message;

static void
shutdown_traffic_light_view(int x)
{
    /* release memory */
    cvReleaseImage(&right_image);

    /* exit module */
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
   // flush_gdk_buffers();
   
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
expose_event(GtkWidget *widget __attribute__((unused)),
             GdkEventExpose *event __attribute__((unused)))
{
    redraw_viewer();
    return 1;
}

static gint
key_press_event(GtkWidget *widget __attribute__((unused)), GdkEventKey *key)
{
    if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK))
        shutdown_traffic_light_view(SIGINT);

    if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
        shutdown_traffic_light_view(SIGINT);

    if (key->state || key->keyval > 255)
        return 1;

    return 1;
}

static gint
key_release_event(GtkWidget *widget __attribute__((unused)),
                  GdkEventButton *key __attribute__((unused)))
{
    return 1;
}

static gint
updateIPC(gpointer *data __attribute__((unused)))
{
    carmen_ipc_sleep(0.01);
    carmen_graphics_update_ipc_callbacks((GdkInputFunction) updateIPC);
    return 1;
}

static void
traffic_light_message_handler(carmen_traffic_light_message *message)
{


	    cv::Mat image;
	    image.create(image_height, image_width, CV_8UC3);
	    image.data = (uchar *) message->traffic_light_image;
	    cv::Mat resized_image ;
	    resized_image.create(window_view_height,window_view_width,CV_8UC3);
	    cv::Size size(window_view_width,window_view_height);
	    resize(image, resized_image, size);
    src_buffer = gdk_pixbuf_new_from_data(resized_image.data, GDK_COLORSPACE_RGB,
         FALSE, 8, window_view_width, window_view_height, window_view_width * 3, NULL, NULL);
    redraw_viewer();

}

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

    gtk_signal_connect(GTK_OBJECT(drawing_area), "expose_event",
                       (GtkSignalFunc) expose_event, NULL);
    gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
                       (GtkSignalFunc) key_press_event, NULL);
    gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
                       (GtkSignalFunc) key_release_event, NULL);

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

    carmen_param_t param_list[] = {
        { (char*) bumblebee_string,(char*) "width", CARMEN_PARAM_INT, &image_width, 0, NULL},
        { (char*) bumblebee_string,(char*) "height", CARMEN_PARAM_INT, &image_height, 0, NULL},
        { (char*) "traffic_light_viewer",(char*) "width", CARMEN_PARAM_INT, &window_view_width, 0, NULL},
        { (char*) "traffic_light_viewer",(char*) "height", CARMEN_PARAM_INT, &window_view_height, 0, NULL}
    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
    return 0;
}

int
main(int argc, char **argv)
{
    /* Connect to IPC Server */
    carmen_ipc_initialize(argc, argv);

    /* Check the param server version */
    carmen_param_check_version(argv[0]);

    /* Register shutdown cleaner handler */
    signal(SIGINT, shutdown_traffic_light_view);

    /* Initialize all the relevant parameters */
    read_parameters(argc, argv);

    /* Allocate my own structures */
    right_image = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_8U, 3);
//        init_traffic_light_viewer();

    /* Subscribe to Traffic Light Services */
    carmen_traffic_light_subscribe(camera, NULL, (carmen_handler_t) traffic_light_message_handler, CARMEN_SUBSCRIBE_LATEST);

    /* Start the graphics windows */
    start_graphics(argc, argv);

    return 0;
}
