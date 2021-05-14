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
#include <carmen/stereo_interface.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

static GtkWidget *horizontal_container;
static GtkWidget *drawing_area_reference_image;
static GdkPixbuf *reference_image;
static GtkWidget *drawing_area_disparity;
static GdkPixbuf *disparity_map_buffer;

static int camera;
static int stereo_width;
static int stereo_height;
static int stereo_disparity;
static int bumblebee_basic_width;
static int bumblebee_basic_height;

guchar *disparity_data = NULL;
guchar *left_image_data = NULL;

#define STEREO_VIEW_MAX_WINDOW_WIDTH 640
#define STEREO_VIEW_MAX_WINDOW_HEIGHT 480
#define STEREO_VIEW_NUM_COLORS 3

static void redraw()
{
    if (bumblebee_basic_width <= STEREO_VIEW_MAX_WINDOW_WIDTH)
    {
        gdk_draw_pixbuf(drawing_area_reference_image->window,
        				drawing_area_reference_image->style->fg_gc[GTK_WIDGET_STATE(drawing_area_reference_image)],
        				reference_image, 0, 0, 0, 0,
                        drawing_area_reference_image->allocation.width,
                        drawing_area_reference_image->allocation.height,
                        GDK_RGB_DITHER_NONE, 0, 0);

        gdk_draw_pixbuf(drawing_area_disparity->window,
        				drawing_area_disparity->style->fg_gc[GTK_WIDGET_STATE(drawing_area_disparity)],
        				disparity_map_buffer, 0, 0, 0, 0,
                        drawing_area_disparity->allocation.width,
                        drawing_area_disparity->allocation.height,
                        GDK_RGB_DITHER_NONE, 0, 0);
    }
    else
    {
        gdk_draw_pixbuf(drawing_area_reference_image->window,
        				drawing_area_reference_image->style->fg_gc[GTK_WIDGET_STATE(drawing_area_reference_image)],
        				reference_image, 0, 0, 0, 0,
        				drawing_area_reference_image->allocation.width,
        				drawing_area_reference_image->allocation.height,
                        GDK_RGB_DITHER_NONE, 0, 0);

        gdk_draw_pixbuf(drawing_area_disparity->window,
        				drawing_area_disparity->style->fg_gc[GTK_WIDGET_STATE(drawing_area_disparity)],
        				disparity_map_buffer, 0, 0, 0, 0,
        				drawing_area_disparity->allocation.width,
        				drawing_area_disparity->allocation.height,
                        GDK_RGB_DITHER_NONE, 0, 0);
    }
}

static void
stereo_message_handler(carmen_simple_stereo_disparity_message *disparity_message)
{
	int i, j, scaled_disparity_value;

	if (bumblebee_basic_width <= STEREO_VIEW_MAX_WINDOW_WIDTH)
	{
		for(i = 0; i < stereo_height; i++)
		{
			for(j = 0; j < stereo_width; j++)
			{
				double disparity_value = disparity_message->disparity[i * stereo_width + j];
				scaled_disparity_value = (unsigned char) (255.0 * (disparity_value) / ((double) stereo_disparity));
				disparity_data[3 * (i * bumblebee_basic_width + j) + 0] = scaled_disparity_value;
				disparity_data[3 * (i * bumblebee_basic_width + j) + 1] = scaled_disparity_value;
				disparity_data[3 * (i * bumblebee_basic_width + j) + 2] = scaled_disparity_value;
			}
		}

		for (i = 0; i< disparity_message->reference_image_size; i++)
			left_image_data[i] = (guchar) disparity_message->reference_image[i];
	}
	else
	{
		IplImage *reference_image = NULL;
		IplImage *depth_image = NULL;

		reference_image = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, STEREO_VIEW_NUM_COLORS);
		depth_image = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, STEREO_VIEW_NUM_COLORS);

		for(i = 0; i < stereo_height; i++)
		{
			for(j = 0; j < stereo_width; j++)
			{
				double disparity_value = disparity_message->disparity[i * stereo_width + j];
				scaled_disparity_value = (unsigned char) (255.0 * ((double) disparity_value) / ((double) stereo_disparity));
				depth_image->imageData[3 * (i * bumblebee_basic_width + j) + 0] = scaled_disparity_value;
				depth_image->imageData[3 * (i * bumblebee_basic_width + j) + 1] = scaled_disparity_value;
				depth_image->imageData[3 * (i * bumblebee_basic_width + j) + 2] = scaled_disparity_value;
			}
		}

		for (i = 0; i< disparity_message->reference_image_size; i++)
			reference_image->imageData[i] = (guchar) disparity_message->reference_image[i];

		IplImage *reference_image_resized = NULL;
		IplImage *depth_image_resized = NULL;

		reference_image_resized = cvCreateImage(cvSize(STEREO_VIEW_MAX_WINDOW_WIDTH, STEREO_VIEW_MAX_WINDOW_HEIGHT), IPL_DEPTH_8U, STEREO_VIEW_NUM_COLORS);
		depth_image_resized = cvCreateImage(cvSize(STEREO_VIEW_MAX_WINDOW_WIDTH, STEREO_VIEW_MAX_WINDOW_HEIGHT), IPL_DEPTH_8U, STEREO_VIEW_NUM_COLORS);

		//resizing the image
		cvResize(reference_image, reference_image_resized, CV_INTER_CUBIC);
		memcpy(left_image_data, (uchar*) reference_image_resized->imageData, reference_image_resized->imageSize);
		cvResize(depth_image, depth_image_resized, CV_INTER_CUBIC);
		memcpy(disparity_data, (uchar*) depth_image_resized->imageData, depth_image_resized->imageSize);

		cvReleaseImage(&reference_image);
		cvReleaseImage(&reference_image_resized);
		cvReleaseImage(&depth_image);
		cvReleaseImage(&depth_image_resized);
	}

	disparity_map_buffer = gdk_pixbuf_new_from_data(disparity_data, GDK_COLORSPACE_RGB,
			FALSE, 8,  STEREO_VIEW_MAX_WINDOW_WIDTH, STEREO_VIEW_MAX_WINDOW_HEIGHT, STEREO_VIEW_MAX_WINDOW_WIDTH * 3, NULL, NULL);

	reference_image = gdk_pixbuf_new_from_data(left_image_data, GDK_COLORSPACE_RGB,
			FALSE, 8,  STEREO_VIEW_MAX_WINDOW_WIDTH, STEREO_VIEW_MAX_WINDOW_HEIGHT, STEREO_VIEW_MAX_WINDOW_WIDTH * 3, NULL, NULL);

	redraw();
}


static void
shutdown_stereo_view(int x)
{
	if (x == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("stereo_view was disconnected.\n");
		exit(1);
	}
}


static gint
updateIPC(gpointer *data __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);
	carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
	return 1;
}


static gint
expose_event(GtkWidget *widget __attribute__ ((unused)), GdkEventExpose *event __attribute__ ((unused)))
{
	redraw();
	return 1;
}


static gint
key_press_event(GtkWidget *widget __attribute__ ((unused)), GdkEventKey *key)
{
	if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK))
		shutdown_stereo_view(SIGINT);

	if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
		shutdown_stereo_view(SIGINT);

	if (key->state || key->keyval > 255)
		return 1;

	return 1;
}


static gint
key_release_event(GtkWidget *widget __attribute__ ((unused)),
		GdkEventButton *key __attribute__ ((unused)))
{
	return 1;
}


static void
start_graphics(int argc, char *argv[])
{
	GtkWidget *main_window;

	gtk_init(&argc, &argv);

	main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title (GTK_WINDOW (main_window), "stereo view");

	horizontal_container = gtk_hbox_new(TRUE, 1);
	drawing_area_reference_image = gtk_drawing_area_new ();
	drawing_area_disparity = gtk_drawing_area_new ();

    if (bumblebee_basic_width <= STEREO_VIEW_MAX_WINDOW_WIDTH)
    {
        gtk_widget_set_usize(drawing_area_reference_image, bumblebee_basic_width, bumblebee_basic_height);
        gtk_widget_set_usize(drawing_area_disparity, bumblebee_basic_width, bumblebee_basic_height);
    }
    else
    {
        gtk_widget_set_usize(drawing_area_reference_image, STEREO_VIEW_MAX_WINDOW_WIDTH, STEREO_VIEW_MAX_WINDOW_HEIGHT);
        gtk_widget_set_usize(drawing_area_disparity, STEREO_VIEW_MAX_WINDOW_WIDTH, STEREO_VIEW_MAX_WINDOW_HEIGHT);
    }

	gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_disparity, TRUE, TRUE, 0);
	gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_reference_image, TRUE, TRUE, 0);

	gtk_container_add(GTK_CONTAINER(main_window), horizontal_container);

	gtk_signal_connect(GTK_OBJECT(drawing_area_disparity), "expose_event",
			(GtkSignalFunc)expose_event, NULL);
	gtk_signal_connect(GTK_OBJECT(drawing_area_reference_image), "expose_event",
			(GtkSignalFunc)expose_event, NULL);

	gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
			(GtkSignalFunc)key_press_event, NULL);
	gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
			(GtkSignalFunc)key_release_event, NULL);

	gtk_widget_add_events(drawing_area_disparity,  GDK_EXPOSURE_MASK
			| GDK_BUTTON_PRESS_MASK
			| GDK_BUTTON_RELEASE_MASK
			| GDK_POINTER_MOTION_MASK
			| GDK_POINTER_MOTION_HINT_MASK
			| GDK_KEY_PRESS_MASK
			| GDK_KEY_RELEASE_MASK);

	gtk_widget_add_events(drawing_area_reference_image,  GDK_EXPOSURE_MASK
			| GDK_BUTTON_PRESS_MASK
			| GDK_BUTTON_RELEASE_MASK
			| GDK_POINTER_MOTION_MASK
			| GDK_POINTER_MOTION_HINT_MASK
			| GDK_KEY_PRESS_MASK
			| GDK_KEY_RELEASE_MASK);

	carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
	gtk_widget_show(drawing_area_disparity);
	gtk_widget_show(drawing_area_reference_image);
	gtk_widget_show(horizontal_container);
	gtk_widget_show(main_window);

	gtk_main();
}


int
read_parameters(int argc, char **argv)
{
	int num_items;

	char bumblebee_string[256];
	char stereo_string[256];

	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", atoi(argv[1]));
	sprintf(stereo_string, "%s%d", "stereo", atoi(argv[1]));

	carmen_param_t param_list[] = {
			{stereo_string, (char *) "width", CARMEN_PARAM_INT, &stereo_width, 0, NULL},
			{bumblebee_string, (char *) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
			{bumblebee_string, (char *) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL},
			{stereo_string, (char *) "height", CARMEN_PARAM_INT, &stereo_height, 0, NULL},
			{stereo_string, (char *) "max_disparity", CARMEN_PARAM_INT, &stereo_disparity, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	if ((argc != 2) && (argc != 3))
		carmen_die("%s: Wrong number of parameters. stereo requires either 1 or 2 parameters and received %d parameter(s). \nUsage:\n %s <camera_number> or\n %s <camera_number[LR]> <camera_number[LR]>", argv[0], argc-1, argv[0], argv[0]);

	camera = atoi(argv[1]);

	read_parameters(argc, argv);

	if (bumblebee_basic_width <= STEREO_VIEW_MAX_WINDOW_WIDTH)
	{
		disparity_data = (guchar *) calloc(3 * bumblebee_basic_width * bumblebee_basic_height , sizeof(guchar));
		left_image_data  = (guchar *) calloc(3 * bumblebee_basic_width * bumblebee_basic_height , sizeof(guchar));
	}
	else
	{
		disparity_data = (guchar *) calloc (STEREO_VIEW_NUM_COLORS * STEREO_VIEW_MAX_WINDOW_WIDTH * STEREO_VIEW_MAX_WINDOW_HEIGHT, sizeof(guchar));
		left_image_data = (guchar *) calloc (STEREO_VIEW_NUM_COLORS * STEREO_VIEW_MAX_WINDOW_WIDTH * STEREO_VIEW_MAX_WINDOW_HEIGHT, sizeof(guchar));
	}

	carmen_stereo_subscribe(camera, NULL, (carmen_handler_t) stereo_message_handler, CARMEN_SUBSCRIBE_LATEST);

	signal(SIGINT, shutdown_stereo_view);

	start_graphics(argc, argv);

	return 0;
}
