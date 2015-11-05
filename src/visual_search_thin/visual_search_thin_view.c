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
#include <carmen/visual_search_thin_interface.h>
#include <carmen/neural_global_localizer_interface.h>
#include <carmen/stereo_util.h>
#include <assert.h>
#include <cairo.h>	//libcairo
#include <gtk/gtk.h>	//gtk interface

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

static carmen_bumblebee_basic_stereoimage_message	visual_search_bumblebee_message;
static carmen_visual_search_thin_train_message		visual_search_training_message;
static carmen_visual_search_thin_test_message		visual_search_testing_message;

static carmen_vector_3D_t line_points[2] = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
static int line_point_index = 0;
static int visual_search_running = 0;
static int camera;
static int size;

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
draw_cross(GtkWidget *drawing, double x_img_point, double y_img_point, int value)
{
	cairo_t * drawing_area = gdk_cairo_create(drawing->window);

	cairo_set_line_width(drawing_area, 2);

	switch(value)
	{
		case 0:
			cairo_set_source_rgba(drawing_area, 255, 0, 0, 1);
			break;
		case 1:
			cairo_set_source_rgba(drawing_area, 255, 255, 0, 1);
			break;
		case 2:
			cairo_set_source_rgba(drawing_area, 0, 0, 255, 1);
			break;
		case 3:
			cairo_set_source_rgba(drawing_area, 0, 255, 0, 1);
			break;
		case 4:
			cairo_set_source_rgba(drawing_area, 255, 0, 255, 1);
			break;
	}

	cairo_move_to(drawing_area, x_img_point, y_img_point - CROSS_LENGHT/2);
	cairo_line_to(drawing_area, x_img_point, y_img_point + CROSS_LENGHT/2);
	cairo_move_to(drawing_area, x_img_point - CROSS_LENGHT/2, y_img_point);
	cairo_line_to(drawing_area, x_img_point + CROSS_LENGHT/2, y_img_point);
	cairo_stroke(drawing_area);

	cairo_destroy(drawing_area);
}

static carmen_vector_3D_p
get_point_3D(carmen_position_p point_right, carmen_position_p point_left)
{
	int disparity;

	disparity = point_left->x - point_right->x;

	return reproject_single_point_to_3D(&camera_params, *point_right, disparity);
}


static void neural_globalpos_handler(carmen_neural_global_localizer_globalpos_message* globalpos_message)
{
	double start, end;

	unsigned char *left_image_message;
	unsigned char *right_image_message;
	int i;


	printf("nova_messagem!\n");

	start = carmen_get_time();

	left_image_message = globalpos_message->test_image;
	right_image_message = globalpos_message->output_image;

	g_object_unref(left_image);	//Unreferencing the gtk object
	g_object_unref(right_image);

	visual_search_running = 0;

	if(!visual_search_running)
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


	for (i = 0; i < 5; i++)
	{
		gdouble x, y, x_, y_;

		x = globalpos_message->saliencies[i].coordinates.x;
		y = globalpos_message->saliencies[i].coordinates.y;

		if(!visual_search_running)
		{
			draw_cross(right_image_drawing_area, x, y, i);

			right_image_point.x = x;
			right_image_point.y = camera_params.height - y;

			visual_search_training_message.reference_points_size = 1;
			visual_search_training_message.reference_points = &right_image_point;
			visual_search_training_message.reference_image_size = size;
			visual_search_training_message.reference_image = right_image_data;
			visual_search_training_message.timestamp = carmen_get_time();
			visual_search_training_message.host = carmen_get_host();

			carmen_visual_search_thin_output_training_message * output_training =
					carmen_visual_search_thin_query_training_message(&visual_search_training_message, 10.0);

			if (output_training)
			{
				visual_search_running = 1;

				visual_search_testing_message.associated_points_size = 0;
				visual_search_testing_message.associated_image_size = size;
				visual_search_testing_message.associated_image = left_image_data;
				visual_search_testing_message.timestamp = carmen_get_time();
				visual_search_testing_message.host = carmen_get_host();
				visual_search_testing_message.scale = 1.0;

				carmen_visual_search_thin_output_message * output_testing =
						carmen_visual_search_thin_query_output_message(&visual_search_testing_message, 10.0);

				if (output_testing)
				{
					visual_search_running = 0;

					left_image_point.x = output_testing->saccade_point.x;
					left_image_point.y = output_testing->saccade_point.y;

					if (left_image_point.x > 0.0 && left_image_point.y > 0.0)
					{
						x_ = left_image_point.x;
						y_ = camera_params.height - left_image_point.y;
						draw_cross(left_image_drawing_area, x_, y_, i);

						printf("%d %d %d %d\n", (int) x, (int) y, (int) x_, (int) y_);

//						carmen_vector_3D_p point_3D = get_point_3D(&right_image_point, &left_image_point);
//						calc_distance(point_3D);
//						free(point_3D);
					}

					free(output_testing);
				}

				free(output_training);
			}
		}
	}

	end  = carmen_get_time();

	printf("time: %f\n", end - start);
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
	
	if(!visual_search_running)
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
}

static void
shutdown_visual_search_view(int x)
{
  if (x == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("Visual Search View was disconnected.\n");
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
  redraw_left_image();
  redraw_right_image();
  return 1;
}


static gint
key_press_event(GtkWidget *widget __attribute__ ((unused)), GdkEventKey *key)
{
  if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK))
    shutdown_visual_search_view(SIGINT);

  if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
    shutdown_visual_search_view(SIGINT);

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

void
calc_distance(const carmen_vector_3D_p point_3D)
{
	if (point_3D)
	{
		line_point_index = (line_point_index+1)%2;
		line_points[line_point_index] = *point_3D;
	}

	double distance = sqrt(
			pow(line_points[0].x-line_points[1].x, 2.0) +
			pow(line_points[0].y-line_points[1].y, 2.0) +
			pow(line_points[0].z-line_points[1].z, 2.0));

	carmen_warn("d = %fm\n", distance);
}

static gboolean
left_button_press_event(GtkWidget *widget __attribute__ ((unused)),
		GdkEventButton *event, gpointer user_data __attribute__ ((unused)))
{
	gdouble x, y;
	guint button;
	
	button = ((GdkEventButton*)event)->button;	// Left Button = 1 ; Right Button = 3
	x = ((GdkEventButton*)event)->x;
	y = ((GdkEventButton*)event)->y;

	if(!visual_search_running && button == 1)
	{
		draw_cross(right_image_drawing_area, x, y, 0);

		right_image_point.x = x;
		right_image_point.y = camera_params.height - y;

		visual_search_training_message.reference_points_size = 1;
		visual_search_training_message.reference_points = &right_image_point;
		visual_search_training_message.reference_image_size = size;
		visual_search_training_message.reference_image = right_image_data;
		visual_search_training_message.timestamp = carmen_get_time();
		visual_search_training_message.host = carmen_get_host();
		
        carmen_visual_search_thin_output_training_message * output_training =
        		carmen_visual_search_thin_query_training_message(&visual_search_training_message, 10.0);

        if (output_training)
        {
        	visual_search_running = 1;

        	visual_search_testing_message.associated_points_size = 0;
    		visual_search_testing_message.associated_image_size = size;
    		visual_search_testing_message.associated_image = left_image_data;
    		visual_search_testing_message.timestamp = carmen_get_time();
    		visual_search_testing_message.host = carmen_get_host();
    		visual_search_testing_message.scale = 1.0;

            carmen_visual_search_thin_output_message * output_testing =
            		carmen_visual_search_thin_query_output_message(&visual_search_testing_message, 10.0);

            if (output_testing)
            {
            	left_image_point.x = output_testing->saccade_point.x;
            	left_image_point.y = output_testing->saccade_point.y;

        		if (left_image_point.x > 0.0 && left_image_point.y > 0.0)
        		{
                	x = left_image_point.x;
                	y = camera_params.height - left_image_point.y;
        			draw_cross(left_image_drawing_area, x, y, 0);

        			carmen_vector_3D_p point_3D = get_point_3D(&right_image_point, &left_image_point);
        			calc_distance(point_3D);
        			free(point_3D);
        		}

        		free(output_testing);
            }

            free(output_training);
        }
	}
	
	return FALSE;
}

static gboolean
right_button_press_event(GtkWidget *widget __attribute__ ((unused)),
		GdkEventButton *event, gpointer user_data __attribute__ ((unused)))
{
	guint button;
	
	button = ((GdkEventButton*)event)->button;	// Left Button = 1 ; Right Button = 3
	
	if(visual_search_running && button == 3)
	{
		visual_search_running = 0;
	}
	
	return FALSE;
}

static void
start_graphics(int argc, char *argv[])
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (main_window), "Visual Search View");

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
       carmen_die("%s: Wrong number of parameters. Visual Search Viewer requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number> ", argv[0], argc-1, argv[0]);

  camera = atoi(argv[1]);

  camera_params = get_stereo_instance(camera, -1, -1);

  alloc_images_memory();

  carmen_visual_search_thin_define_message_query_train();
  carmen_visual_search_thin_define_message_query_test();
  
  //Subscribes to the bumblebee messages
//  if(camera != -1)
//  	carmen_bumblebee_basic_subscribe_stereoimage(camera, &visual_search_bumblebee_message, (carmen_handler_t)bumblebee_image_handler, CARMEN_SUBSCRIBE_LATEST);

  carmen_neural_global_localizer_subscribe_globalpos_message(NULL, (carmen_handler_t) neural_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

  signal(SIGINT, shutdown_visual_search_view);

  start_graphics(argc, argv);

  return 0;
}
