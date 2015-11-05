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

#include <carmen/carmen_graphics.h>	//must come first
#include <carmen/vergence_interface.h>
#include <carmen/stereo_util.h>

#include <cairo.h>	//libcairo
#include <gtk/gtk.h>	//gtk interface

#include <opencv/cv.h>
#include <opencv/highgui.h>

static GtkWidget *horizontal_container;

static GtkWidget *left_image_drawing_area;
static GdkPixbuf *left_image;
guchar *left_image_data = NULL;

static GtkWidget *right_image_drawing_area;
static GdkPixbuf *right_image;
guchar *right_image_data = NULL;

static stereo_util camera_params;
static carmen_position_t right_image_point;
static carmen_position_t left_image_point;

static carmen_bumblebee_basic_stereoimage_message	bumblebee_message;
static carmen_vergence_train_message	vergence_train_message;
static carmen_vergence_test_message		vergence_test_message;

static carmen_vector_3D_t line_points[2] = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
static int line_point_index = 0;
static int vergence_running = 0;
static int camera;
static int size;

CvPoint2D32f right_corners[180];
CvPoint2D32f left_corners[180];
int disparity[180];

static gdouble right_image_x, image_y;
static int current_disparity;

#define CROSS_LENGHT	10

static void 
redraw_right_image()
{
  gdk_draw_pixbuf(right_image_drawing_area->window,
		  right_image_drawing_area->style->fg_gc[GTK_WIDGET_STATE (right_image_drawing_area)],
		  right_image, 0, 0, 0, 0,
 		  right_image_drawing_area->allocation.width,
 		  right_image_drawing_area->allocation.height,
 		  GDK_RGB_DITHER_NONE, 0, 0);
}

static void 
redraw_left_image()
{
  gdk_draw_pixbuf(left_image_drawing_area->window,
		  left_image_drawing_area->style->fg_gc[GTK_WIDGET_STATE (left_image_drawing_area)],
		  left_image, 0, 0, 0, 0,
 		  left_image_drawing_area->allocation.width,
 		  left_image_drawing_area->allocation.height,
 		  GDK_RGB_DITHER_NONE, 0, 0);
}

// Draws a cross at the X-Y selected position
static void
draw_cross(GtkWidget *drawing, double x_img_point, double y_img_point)
{
	cairo_t * drawing_area = gdk_cairo_create(drawing->window);

	cairo_set_line_width(drawing_area, 2);
	cairo_set_source_rgba(drawing_area, 255, 0, 0, 1);

	cairo_move_to(drawing_area, x_img_point, y_img_point - CROSS_LENGHT/2);
	cairo_line_to(drawing_area, x_img_point, y_img_point + CROSS_LENGHT/2);
	cairo_move_to(drawing_area, x_img_point - CROSS_LENGHT/2, y_img_point);
	cairo_line_to(drawing_area, x_img_point + CROSS_LENGHT/2, y_img_point);

	cairo_stroke(drawing_area);

	cairo_destroy(drawing_area);
}

carmen_vector_3D_p
get_point_3D(carmen_position_p point_right, carmen_position_p point_left)
{
	int disparity;

	disparity = point_left->x - point_right->x;

	return reproject_single_point_to_3D(&camera_params, *point_right, disparity);
}

void
calc_distance(const carmen_vector_3D_p point_3D)
{
	if (point_3D)
	{
		line_point_index = (line_point_index+1)%2;
		line_points[line_point_index] = *point_3D;
		carmen_warn("x = %fm\n", point_3D->x);
		carmen_warn("y = %fm\n", point_3D->y);
		carmen_warn("z = %fm\n", point_3D->z);
	}

	double distance = sqrt(
			pow(line_points[0].x-line_points[1].x, 2.0) +
			pow(line_points[0].y-line_points[1].y, 2.0) +
			pow(line_points[0].z-line_points[1].z, 2.0));

	carmen_warn("d = %fm\n", distance);
}

static int
find_corners(IplImage *rectified_image, CvPoint2D32f *corners, int corners_h, int corners_v)
{
	int corners_count = 0;

	CvSize imgSize = cvSize(rectified_image->width, rectified_image->height);

	IplImage* rectified_image_gray = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);

	cvConvertImage(rectified_image, rectified_image_gray, CV_BGR2GRAY);

	cvFindChessboardCorners(
			rectified_image_gray,
			cvSize( corners_h, corners_v ),
			corners, &corners_count,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
	);

	//Get Subpixel accuracy on those corners
	cvFindCornerSubPix(rectified_image_gray, corners, corners_count,
			cvSize(11,11),cvSize(-1,-1), cvTermCriteria(
					CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 0.01 ));

	return corners_count;
}

static int
run_vergence(int x, int y)
{
	int disparity = 0;

	right_image_point.x = x;
	right_image_point.y = camera_params.height - y;

	vergence_train_message.reference_points_size = 1;
	vergence_train_message.reference_points = &right_image_point;
	vergence_train_message.reference_image_size = size;
	vergence_train_message.reference_image = right_image_data;
	vergence_train_message.timestamp = carmen_get_time();
	vergence_train_message.host = carmen_get_host();

    carmen_vergence_train_output_message * output_training =
    		carmen_vergence_query_train_message(&vergence_train_message, 60.0);

    if (output_training)
    {
    	vergence_running = 1;

    	vergence_test_message.associated_points_size = 0;
    	vergence_test_message.associated_image_size = size;
    	vergence_test_message.associated_image = left_image_data;
    	vergence_test_message.timestamp = carmen_get_time();
    	vergence_test_message.host = carmen_get_host();

        carmen_vergence_test_output_message * output_testing =
        		carmen_vergence_query_test_message(&vergence_test_message, 60.0);

        if (output_testing)
        {
        	left_image_point.x = output_testing->vergence_point.x;
        	left_image_point.y = output_testing->vergence_point.y;

    		if (left_image_point.x > 0.0 && left_image_point.y > 0.0)
    		{
    			disparity = left_image_point.x - right_image_point.x;
    		}

    		free(output_testing);
        }

        free(output_training);
    }
	vergence_running = 0;
	return disparity;
}

void
copy_RGB_image_to_BGR_image(unsigned char *original, IplImage *copy, int nchannels)
{
	int i, j;

	for(i = 0; i < copy->height; i++)
	{
		unsigned char* data = (unsigned char*)copy->imageData + (i*copy->widthStep);
		if(nchannels==3)
			for(j = 0; j < copy->width; j++)
			{
				data[nchannels*j+2] = original[nchannels*(i*copy->width+j)+0];
				data[nchannels*j+1] = original[nchannels*(i*copy->width+j)+1];
				data[nchannels*j+0] = original[nchannels*(i*copy->width+j)+2];
			}
		else
			for(j = 0; j < copy->width; j++)
			{
				data[j] = original[i*copy->width+j];
			}
	}
}

static void
test_vergence()
{
	int corners_h = 17;
	int corners_v = 9;
	IplImage *right_image = cvCreateImage(cvSize(camera_params.width, camera_params.height), IPL_DEPTH_8U, 3);
	IplImage *left_image = cvCreateImage(cvSize(camera_params.width, camera_params.height), IPL_DEPTH_8U, 3);

	copy_RGB_image_to_BGR_image(right_image_data, right_image, 3);
	copy_RGB_image_to_BGR_image(left_image_data, left_image, 3);

	find_corners(right_image, right_corners, corners_h, corners_v);
	find_corners(left_image, left_corners, corners_h, corners_v);

	int i;
	for(i=0;i<corners_h*corners_v;i++)
	{
		disparity[i] = run_vergence(
				(int)rint(right_corners[i].x),
				(int)rint(right_corners[i].y));
	}

	float sum = 0.0f;
	float error = 0.0f;
	float count = 0.0f;
	float real_disparity;
	for(i=0;i<corners_h*corners_v;i++)
	{
		real_disparity = left_corners[i].x - right_corners[i].x;
		error = (float)disparity[i] - real_disparity;
		sum += error * error;
		count++;

		carmen_warn("%d, %f, %f,",
				i, left_corners[i].x , left_corners[i].y);

		carmen_warn("%f, %f, %d, %f, %f, %f\n",
				right_corners[i].x , right_corners[i].y,
				disparity[i], real_disparity, error, error*error);
	}
	carmen_warn ("RMS error (%f / %f): %f\n", sum, count, sqrt(sum / count));

	cvReleaseImage(&right_image);
	cvReleaseImage(&left_image);
}

static void
bumblebee_image_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	unsigned char *left_image_message; 
	unsigned char *right_image_message;
	int i;

	assert (size == stereo_image->image_size);

	left_image_message = stereo_image->raw_left;
	right_image_message = stereo_image->raw_right;
	
	g_object_unref(left_image);	//Unreferencing the gtk object
	g_object_unref(right_image);
	
	if(!vergence_running)
	{
		for (i = 0; i < size; i++)			// 3*size -> total number of channels
			left_image_data[i] = (guchar) left_image_message[i];

		for (i = 0; i < size; i++)			// Static image zeroing
			right_image_data[i] = (guchar) right_image_message[i];
	
		left_image = gdk_pixbuf_new_from_data(left_image_data, GDK_COLORSPACE_RGB,
		     	   	   	   	   	   FALSE, 8,  camera_params.width, camera_params.height, camera_params.width * 3, NULL, NULL);
		     	   	   	   	   	   
		right_image = gdk_pixbuf_new_from_data(right_image_data, GDK_COLORSPACE_RGB,
		     	   	   	   	   	   FALSE, 8,  camera_params.width, camera_params.height, camera_params.width * 3, NULL, NULL);

		redraw_left_image();

		redraw_right_image();
	}

	if (current_disparity > 0)
	{
		draw_cross(right_image_drawing_area, right_image_x, image_y);
		draw_cross(left_image_drawing_area, right_image_x + current_disparity, image_y);
	}
}

static void
shutdown_vergence_view(int x)
{
  if (x == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("Vergence View was disconnected.\n");
    exit(1);
  }
}


static gint
updateIPC(gpointer *data __attribute__ ((unused)))
{
  //carmen_ipc_sleep(0.01);
  IPC_handleMessage(0.01);
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  return 1;
}


static gint
expose_event(GtkWidget *widget __attribute__ ((unused)), GdkEventExpose *event __attribute__ ((unused)))
{
  redraw_left_image();
  redraw_right_image();
  return 1;
}


static gint
key_press_event(GtkWidget *widget __attribute__ ((unused)), GdkEventKey *key)
{
  if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK))
    shutdown_vergence_view(SIGINT);

  if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
    shutdown_vergence_view(SIGINT);

  if (toupper(key->keyval) == 'T' && (key->state & GDK_CONTROL_MASK))
    test_vergence();

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

static gboolean
left_button_press_event(GtkWidget *widget __attribute__ ((unused)),
		GdkEventButton *event, gpointer user_data __attribute__ ((unused)))
{
	guint button;
	
	button = ((GdkEventButton*)event)->button;	// Left Button = 1 ; Right Button = 3
	right_image_x = ((GdkEventButton*)event)->x;
	image_y = ((GdkEventButton*)event)->y;

	if(!vergence_running && button == 1)
	{
		current_disparity = run_vergence(right_image_x, image_y);

		carmen_vector_3D_p point_3D = get_point_3D(&right_image_point, &left_image_point);
		calc_distance(point_3D);
		free(point_3D);
	}
	
	return FALSE;
}

static gboolean
right_button_press_event(GtkWidget *widget __attribute__ ((unused)),
		GdkEventButton *event, gpointer user_data __attribute__ ((unused)))
{
	guint button;
	
	button = ((GdkEventButton*)event)->button;	// Left Button = 1 ; Right Button = 3
	
	if(!vergence_running && button == 3)
	{
		//test_vergence();
	}
	
	return FALSE;
}

static void
start_graphics(int argc, char *argv[])
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (main_window), "Vergence View");

  horizontal_container = gtk_hbox_new(TRUE, 1);
  left_image_drawing_area = gtk_drawing_area_new ();
  right_image_drawing_area = gtk_drawing_area_new ();

  gtk_widget_set_size_request (left_image_drawing_area, camera_params.width, camera_params.height);
  gtk_widget_set_size_request (right_image_drawing_area, camera_params.width, camera_params.height);

  gtk_box_pack_start(GTK_BOX(horizontal_container), left_image_drawing_area, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(horizontal_container), right_image_drawing_area, TRUE, TRUE, 0);

  gtk_container_add(GTK_CONTAINER(main_window), horizontal_container);

  gtk_signal_connect(GTK_OBJECT(right_image_drawing_area), "expose_event",
 		     (GtkSignalFunc)expose_event, NULL);
  gtk_signal_connect(GTK_OBJECT(left_image_drawing_area), "expose_event",
 		     (GtkSignalFunc)expose_event, NULL);
  
  gtk_signal_connect(GTK_OBJECT(right_image_drawing_area), "button_press_event", 
 		     (GtkSignalFunc)right_button_press_event, NULL);
  gtk_signal_connect(GTK_OBJECT(right_image_drawing_area), "button_press_event",
 		     (GtkSignalFunc)left_button_press_event, NULL);


  gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
 		     (GtkSignalFunc)key_press_event, NULL);
  gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
 		     (GtkSignalFunc)key_release_event, NULL);

  gtk_widget_add_events(right_image_drawing_area,  GDK_EXPOSURE_MASK
   			| GDK_BUTTON_PRESS_MASK	
   			| GDK_BUTTON_RELEASE_MASK
   			| GDK_POINTER_MOTION_MASK
   			| GDK_POINTER_MOTION_HINT_MASK
   			| GDK_KEY_PRESS_MASK
   			| GDK_KEY_RELEASE_MASK);

  gtk_widget_add_events(left_image_drawing_area,  GDK_EXPOSURE_MASK
   			| GDK_BUTTON_PRESS_MASK
   			| GDK_BUTTON_RELEASE_MASK
   			| GDK_POINTER_MOTION_MASK
   			| GDK_POINTER_MOTION_HINT_MASK
   			| GDK_KEY_PRESS_MASK
   			| GDK_KEY_RELEASE_MASK);

  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  gtk_widget_show(left_image_drawing_area);
  gtk_widget_show(right_image_drawing_area);
  gtk_widget_show(horizontal_container);
  gtk_widget_show(main_window);

  gtk_main();
}

void
alloc_images_memory()
{
	size = 3 * camera_params.width * camera_params.height;

	left_image_data = (guchar *) calloc(size , sizeof(guchar));
	right_image_data = (guchar *) calloc(size , sizeof(guchar));
  
	carmen_test_alloc(left_image_data);
	carmen_test_alloc(right_image_data);
}

int
main(int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);

  carmen_param_check_version(argv[0]);

  if (argc != 2)
       carmen_die("%s: Wrong number of parameters. Vergence Viewer requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number> ", argv[0], argc-1, argv[0]);

  camera = atoi(argv[1]);

  camera_params = get_stereo_instance(camera, -1, -1);

  alloc_images_memory();

  carmen_vergence_define_message_query_train();

  carmen_vergence_define_message_query_test();
  
  //Subscribes to the bumblebee messages
  if(camera != -1)
  	carmen_bumblebee_basic_subscribe_stereoimage(camera, &bumblebee_message, (carmen_handler_t)bumblebee_image_handler, CARMEN_SUBSCRIBE_LATEST);

  signal(SIGINT, shutdown_vergence_view);

  start_graphics(argc, argv);

  return 0;
}
