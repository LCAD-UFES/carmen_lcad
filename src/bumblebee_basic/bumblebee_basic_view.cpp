/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen_graphics.h>

#include <cstdio>
#include <iostream>
#include <vector>
#include <list>
#include <string>

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/fused_odometry_messages.h>
#include <carmen/fused_odometry_interface.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <carmen/fused_odometry_interface.h>

using namespace std;

#define BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH 640
#define BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT 480
#define BUMBLEBEE_BASIC_VIEW_NUM_COLORS 3

static GtkWidget *horizontal_container;
static GtkWidget *drawing_area_left;
static GdkPixbuf *image_left;
static GtkWidget *drawing_area_right;
static GdkPixbuf *image_right;

static int received_image = 0;
static void redraw(void);

static int bumblebee_basic_width;
static int bumblebee_basic_height;

static unsigned char *data_left = NULL;
static unsigned char *data_right = NULL;

static carmen_bumblebee_basic_stereoimage_message last_message;

static int msg_fps = 0, msg_last_fps = 0; //message fps
static int disp_fps = 0, disp_last_fps = 0; //display fps

static vector<carmen_bumblebee_basic_stereoimage_message> lista_imagens;


static void
process_image(carmen_bumblebee_basic_stereoimage_message *msg)
{
    IplImage *src_image_left = NULL;
    IplImage *src_image_right = NULL;
    IplImage *resized_image_left = NULL;
    IplImage *resized_image_right = NULL;

    static char msg_fps_string[256];
    static char disp_fps_string[256];

    int scaled_width, scaled_height;

    scaled_width = floor((double) drawing_area_left->allocation.width / 8.0) * 8.0;
    scaled_height = (double) (scaled_width / (double) msg->width) * msg->height;

    src_image_left = cvCreateImage(cvSize(msg->width, msg->height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);
    src_image_right = cvCreateImage(cvSize(msg->width, msg->height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);

    if (resized_image_right)
    {
        cvReleaseImage(&resized_image_left);
        cvReleaseImage(&resized_image_right);
    }

    resized_image_left = cvCreateImage(cvSize(scaled_width, scaled_height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);
    resized_image_right = cvCreateImage(cvSize(scaled_width, scaled_height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);

    src_image_left->imageData = (char*) msg->raw_left;
    src_image_right->imageData = (char*) msg->raw_right;

    //resizing the image
    cvResize(src_image_left, resized_image_left, CV_INTER_CUBIC);
    cvResize(src_image_right, resized_image_right, CV_INTER_CUBIC);

    CvFont font;
    sprintf(msg_fps_string, "MSG_FPS: %d", msg_last_fps);
    sprintf(disp_fps_string, "DISP_FPS: %d", disp_last_fps);
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.7, 0.7, 0, 1.0, CV_AA);
    cvPutText(resized_image_left, msg_fps_string, cvPoint(10, 30), &font, cvScalar(255, 255, 0, 0));
    cvPutText(resized_image_right, disp_fps_string, cvPoint(10, 30), &font, cvScalar(255, 255, 0, 0));

    image_left = gdk_pixbuf_new_from_data((guchar *) resized_image_left->imageData, GDK_COLORSPACE_RGB,
                                          FALSE, 8 * sizeof (unsigned char), scaled_width,
                                          scaled_height, scaled_width * BUMBLEBEE_BASIC_VIEW_NUM_COLORS,
                                          NULL, NULL);


    image_right = gdk_pixbuf_new_from_data((guchar *) resized_image_right->imageData, GDK_COLORSPACE_RGB,
                                           FALSE, 8 * sizeof (unsigned char), scaled_width,
                                           scaled_height, scaled_width * BUMBLEBEE_BASIC_VIEW_NUM_COLORS,
                                           NULL, NULL);

    redraw();
    
    cvReleaseImage(&resized_image_left);
    cvReleaseImage(&resized_image_right);
    cvReleaseImage(&src_image_left);
    cvReleaseImage(&src_image_right);
}


static void
image_handler(carmen_bumblebee_basic_stereoimage_message* image_msg)
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

    process_image(image_msg);
}


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
}


static void
shutdown_camera_view(int x)
{
    data_left = NULL;
    data_right = NULL;
    free(data_left);
    free(data_right);

    if (x == SIGINT)
    {
        carmen_ipc_disconnect();
        printf("Disconnected from robot.\n");
        exit(1);
    }
}


static gint
updateIPC(gpointer *data __attribute__((unused)))
{
    carmen_ipc_sleep(0.01);
    carmen_graphics_update_ipc_callbacks((GdkInputFunction) updateIPC);

    return 1;
}


static gint
expose_event(GtkWidget *widget __attribute__((unused)),
             GdkEventExpose *event __attribute__((unused)))
{
    if (received_image)
        process_image(&last_message);

    return 1;
}


static gint
key_press_event(GtkWidget *widget __attribute__((unused)),
                GdkEventKey *key)
{
    if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK))
        shutdown_camera_view(SIGINT);

    if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
        shutdown_camera_view(SIGINT);

    if (toupper(key->keyval) == 'S' && (key->state & GDK_CONTROL_MASK))
        save_camera_images();

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


static void
redraw(void)
{
    /* Make sure data structures are all the right size. */

    if (!received_image)

        return;

    int width, height;

    width = gdk_pixbuf_get_width(image_left);
    height = gdk_pixbuf_get_height(image_left);

    gdk_draw_pixbuf(drawing_area_left->window,
                    drawing_area_left->style->fg_gc[GTK_WIDGET_STATE(drawing_area_left)],
                    image_left, 0, 0, 0, 0,
                    width,
                    height,
                    GDK_RGB_DITHER_NONE, 0, 0);

    gdk_draw_pixbuf(drawing_area_right->window,
                    drawing_area_right->style->fg_gc[GTK_WIDGET_STATE(drawing_area_right)],
                    image_right, 0, 0, 0, 0,
                    width,
                    height,
                    GDK_RGB_DITHER_NONE, 0, 0);
}


GdkPixbuf *
create_pixbuf(const gchar * filename)
{
    GdkPixbuf *pixbuf;
    GError *error = NULL;
    pixbuf = gdk_pixbuf_new_from_file(filename, &error);
    if (!pixbuf)
    {

        fprintf(stderr, "%s\n", error->message);
        g_error_free(error);
    }

    return pixbuf;
}


static void
start_graphics(int argc, char *argv[])
{

    GtkWidget *main_window;
    char window_name[100];

    gtk_init(&argc, &argv);

    main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    sprintf(window_name, "Bumblebee Camera %s", argv[1]);
    gtk_window_set_title(GTK_WINDOW(main_window), window_name);
    std::string aux = getenv("CARMEN_HOME");
    aux.append("/data/gui/camera.png");
    gtk_window_set_icon(GTK_WINDOW(main_window), create_pixbuf(aux.c_str()));

    horizontal_container = gtk_hbox_new(TRUE, 0);
    drawing_area_left = gtk_drawing_area_new();
    drawing_area_right = gtk_drawing_area_new();

    //	if (bumblebee_basic_width <= BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH)
    //	{
    //		gtk_widget_set_usize(drawing_area_left, bumblebee_basic_width, bumblebee_basic_height);
    //		gtk_widget_set_usize(drawing_area_right, bumblebee_basic_width, bumblebee_basic_height);
    //	}
    //	else
    //	{
    //		gtk_widget_set_usize(drawing_area_left, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT);
    //		gtk_widget_set_usize(drawing_area_right, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT);
    //	}

    gtk_widget_set_size_request(horizontal_container, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH * 2, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT);


    gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_left, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_right, TRUE, TRUE, 0);

    gtk_container_add(GTK_CONTAINER(main_window), horizontal_container);

    gtk_signal_connect(GTK_OBJECT(drawing_area_left), "expose_event",
                       (GtkSignalFunc) expose_event, NULL);
    gtk_signal_connect(GTK_OBJECT(drawing_area_right), "expose_event",
                       (GtkSignalFunc) expose_event, NULL);

    gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
                       (GtkSignalFunc) key_press_event, NULL);
    gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
                       (GtkSignalFunc) key_release_event, NULL);

    gtk_widget_add_events(drawing_area_left, GDK_EXPOSURE_MASK
                          | GDK_BUTTON_PRESS_MASK
                          | GDK_BUTTON_RELEASE_MASK
                          | GDK_POINTER_MOTION_MASK
                          | GDK_POINTER_MOTION_HINT_MASK
                          | GDK_KEY_PRESS_MASK
                          | GDK_KEY_RELEASE_MASK);

    gtk_widget_add_events(drawing_area_right, GDK_EXPOSURE_MASK
                          | GDK_BUTTON_PRESS_MASK
                          | GDK_BUTTON_RELEASE_MASK
                          | GDK_POINTER_MOTION_MASK
                          | GDK_POINTER_MOTION_HINT_MASK
                          | GDK_KEY_PRESS_MASK
                          | GDK_KEY_RELEASE_MASK);

    carmen_graphics_update_ipc_callbacks((GdkInputFunction) updateIPC);
    gtk_widget_show(drawing_area_left);
    gtk_widget_show(drawing_area_right);
    gtk_widget_show(horizontal_container);
    gtk_widget_show(main_window);

    gtk_main();
}


static int
read_parameters(int argc, char **argv, int camera)
{
    int num_items;

    std::string bumblebee_string = "bumblebee_basic";
    std::ostringstream myStream;
    myStream << camera << std::flush;
    bumblebee_string.append(myStream.str());
    std::string camera_string = "camera";
    camera_string.append(myStream.str());

    carmen_param_t param_list[] = {
    		{(char*) bumblebee_string.c_str(), (char*) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
    		{(char*) bumblebee_string.c_str(), (char*) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL},
    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
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
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);
    read_parameters(argc, argv, camera);

    if (bumblebee_basic_width <= BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH)
    {
        data_left = (unsigned char *) calloc(bumblebee_basic_width * bumblebee_basic_height * BUMBLEBEE_BASIC_VIEW_NUM_COLORS, sizeof (unsigned char));
        data_right = (unsigned char *) calloc(bumblebee_basic_width * bumblebee_basic_height * BUMBLEBEE_BASIC_VIEW_NUM_COLORS, sizeof (unsigned char));
    }
    else
    {
        data_left = (unsigned char *) calloc(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH * BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT * BUMBLEBEE_BASIC_VIEW_NUM_COLORS, sizeof (unsigned char));
        data_right = (unsigned char *) calloc(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH * BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT * BUMBLEBEE_BASIC_VIEW_NUM_COLORS, sizeof (unsigned char));
    }

    signal(SIGINT, shutdown_camera_view);

    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
    start_graphics(argc, argv);

    return 0;
}
