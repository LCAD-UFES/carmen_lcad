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
#include <carmen/tracker_interface.h>
#include <carmen/stereo_interface.h>
//#include <carmen/neural_global_localizer_interface.h>
#include <carmen/stereo_util.h>
#include <assert.h>
#include <cairo.h>	//libcairo
#include <gtk/gtk.h>	//gtk interface
#include <vector>

static GtkWidget *horizontal_container;

static GtkWidget *left_image_drawing_area;
static GdkPixbuf *left_image;
guchar *left_image_data = NULL;

static GtkWidget *right_image_drawing_area;
static GdkPixbuf *right_image;
guchar *right_image_data = NULL;

static stereo_util camera_params;
static carmen_position_t right_image_point;
static carmen_position_t right_image_point0;
static carmen_position_t right_image_point1;

static carmen_bumblebee_basic_stereoimage_message	visual_search_bumblebee_message;
static carmen_tracker_train_message		visual_search_training_message;
static carmen_tracker_test_message		visual_search_testing_message_right;
carmen_tracker_output_training_message * output_training;

static carmen_vector_3D_t line_points[2] = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
static int line_point_index = 0;
static int visual_search_running = 0;
static int camera;
static int size;
static int is_trained = 0;
static gdouble x_train1 = -1;
static gdouble y_train1 = -1;
static gdouble x_train2 = -1;
static gdouble y_train2 = -1;

std::vector<carmen_simple_stereo_disparity_message>  global_simple_stereo_disparity_message;

#define CROSS_LENGHT	10



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

void
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
void
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

// Draws a retangle
void
draw_retangle(GtkWidget *drawing, double x1, double y1, double x2, double y2, int value)
{
	cairo_t * drawing_area = gdk_cairo_create(drawing->window);

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

	cairo_rectangle(drawing_area, x1, y1, x2 - x1, y2 - y1);
	cairo_set_line_width(drawing_area, 2);
	cairo_set_line_join(drawing_area, CAIRO_LINE_JOIN_MITER); 
	cairo_stroke(drawing_area);

	cairo_destroy(drawing_area);
}


static carmen_position_t
getPointCloundFromDisparity(float * disparityMap, carmen_position_t init_point, carmen_position_t final_point, stereo_util camera_params)
{
	double pointDisparity;
	carmen_position_t new_point = {9999.0, 0.0};
	for (int y = (int)init_point.y; y < (int) final_point.y; y++)
	{
		for (int x = (int)init_point.x; x < (int) final_point.x; x++)
		{
			pointDisparity = disparityMap[y * camera_params.width + x];
			if (pointDisparity <= 0.0)
				continue;
			pointDisparity = fmax(pointDisparity, 0.1f);
		      carmen_position_t P_r;
		      P_r.x = x;
		      P_r.y = y;
		    carmen_vector_3D_p P_world = reproject_single_point_to_3D(&camera_params, P_r, pointDisparity);

		    if (new_point.x > P_world->x)
		    {
				new_point.x = P_world->x;
				new_point.y = P_world->y;
		    }
			free(P_world);
		}
	}
	return (new_point);
}


static carmen_position_t get_point_from_window_disparity(double image_timestamp, carmen_position_t init_point, carmen_position_t final_point)
{
	carmen_position_t new_point = {0.0, 0.0};
	for (unsigned int i = 0; i < global_simple_stereo_disparity_message.size(); i++)
	{
		if (fabs(image_timestamp - global_simple_stereo_disparity_message[i].timestamp) < 0.1)
		{
			new_point = getPointCloundFromDisparity(global_simple_stereo_disparity_message[i].disparity, init_point, final_point, camera_params);
			return new_point;
		}
	}

	return new_point;
}


void
carmen_stereo_disparity_handler(carmen_simple_stereo_disparity_message* message)
{
	global_simple_stereo_disparity_message.push_back(*message);
}



static void
bumblebee_image_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	
	unsigned char *right_image_message;
	static double height_in_train = -1.0;
	static double width_in_train = -1.0;
	static double dynamic_scale_factor_init = -1.0;

	int i;
	int largura = 0;
	int altura = 0;
	float scale = 0;

	right_image_message = stereo_image->raw_right;
	
	g_object_unref(right_image);//Unreferencing the gtk object
	
	if(!visual_search_running)
	{

		for (i = 0; i < size; i++)			// Static image zeroing
			right_image_data[i] = (guchar) right_image_message[i];
		     	   	   	   	   	   
		right_image = gdk_pixbuf_new_from_data(right_image_data, GDK_COLORSPACE_RGB,
		     	   	   	   	   	   FALSE, 8,  camera_params.width, camera_params.height, camera_params.width * 3, NULL, NULL);

		redraw_right_image();
	}

	if(!visual_search_running)
	{
		if ((x_train1 != -1)&&(y_train1 != -1)&&(x_train2 != -1)&&(y_train2 != -1))
		{
			right_image_point0.x = (int)(x_train1);
			right_image_point0.y = camera_params.height - (int)(y_train1);
			right_image_point1.x = (int)(x_train2);
			right_image_point1.y = camera_params.height - (int)(y_train2);

			height_in_train = fabs(y_train2 - y_train1);
			width_in_train = fabs(x_train2 - x_train1);

			if (is_trained == 0){
				//treina imagem direita
				visual_search_training_message.reference_points_size = 2;
				visual_search_training_message.reference_points = (carmen_position_t *) malloc (2 * sizeof(carmen_position_t));
				visual_search_training_message.reference_points[0] = right_image_point0;
				visual_search_training_message.reference_points[1] = right_image_point1;
				visual_search_training_message.reference_image_size = size;
				visual_search_training_message.reference_image = right_image_data;
				visual_search_training_message.timestamp = carmen_get_time();
				visual_search_training_message.host = carmen_get_host();

				output_training = carmen_tracker_query_training_message(&visual_search_training_message, 10.0);
				is_trained = 1;
			}

			if (output_training)

			if (is_trained == 1)
			{
				visual_search_running = 1;
				//testa imagem direita
				visual_search_testing_message_right.associated_points_size = 0;
				visual_search_testing_message_right.associated_image_size = size;
				visual_search_testing_message_right.associated_image = right_image_data;
				visual_search_testing_message_right.timestamp = carmen_get_time();
				visual_search_testing_message_right.host = carmen_get_host();
				visual_search_testing_message_right.scale = 1.0;

				carmen_tracker_output_message * output_testing_right =
					carmen_tracker_query_output_message(&visual_search_testing_message_right, 10.0);

				if (output_testing_right)
				{

					right_image_point.x = output_testing_right->saccade_point.x;
					right_image_point.y = output_testing_right->saccade_point.y;
					scale = output_testing_right->measured_scale_factor;

					if (dynamic_scale_factor_init == -1.0)
					{
						dynamic_scale_factor_init = scale;
					}

					if (right_image_point.x > 0.0 && right_image_point.y > 0.0)
					{
						altura = (int) ((dynamic_scale_factor_init / scale) * height_in_train + 0.5);
						largura = (int) ((dynamic_scale_factor_init / scale) * width_in_train + 0.5);
						altura = abs(altura);
						largura = abs(largura);

						//desenhando o resultado na imagem direita
						/*x_ = right_image_point.x;
						y_ = camera_params.height - right_image_point.y;
						draw_cross(right_image_drawing_area, x_, y_, 4);
						draw_retangle(right_image_drawing_area, x_ - largura/2, y_ - altura/2, x_ + largura/2, y_ + altura/2, 0);
						*/

						carmen_position_t init_point, final_point;
						init_point.x = right_image_point.x - largura/2;
						final_point.x = right_image_point.x + largura/2;
						init_point.y = (camera_params.height - right_image_point.y - altura/2);
						final_point.y = (camera_params.height - right_image_point.y + altura/2);

						carmen_position_t new_position_point = get_point_from_window_disparity(stereo_image->timestamp, init_point, final_point);
						//printf("init_point = %f, final_point = %f\n", init_point, final_point);

						//printf("new_position_point.x %.2f new_position_point.y %.2f\n",new_position_point.x,new_position_point.y);
						position_message_handler(new_position_point.x, new_position_point.y, stereo_image->timestamp);

					}

					free(output_testing_right);
				}
				visual_search_running = 0;
			}
		}
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

static void
updateIPC(gpointer data __attribute__ ((unused)), gint source __attribute__ ((unused)), GdkInputCondition condition __attribute__ ((unused)))
{
  carmen_ipc_sleep(0.01);
  carmen_graphics_update_ipc_callbacks(updateIPC);
}

static gint
expose_event(GtkWidget *widget __attribute__ ((unused)), GdkEventExpose *event __attribute__ ((unused)))
{
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


void button_was_clicked(GtkWidget *widget __attribute__ ((unused)), GdkEventButton *event, gpointer user_data __attribute__ ((unused)))
{
	gdouble x_aux;
	gdouble y_aux;

	if (x_train1 == -1)
	{
		x_train1 = ((GdkEventButton*)event)->x;
	}
	else
	{
		if (x_train2 == -1)
		{
			x_train2 = ((GdkEventButton*)event)->x;
			if (x_train2 == x_train1) x_train2 = -1;
		}
	}	

	if (y_train1 == -1)
	{
		y_train1 = ((GdkEventButton*)event)->y;
	}
	else
	{	
		if (y_train2 == -1)
		{
			y_train2 = ((GdkEventButton*)event)->y;
			if (y_train2 == y_train1) y_train2 = -1;
		}
	}

	if ((x_train1 != -1)&&(y_train1 != -1)&&(x_train2 != -1)&&(y_train2 != -1))
	{
		if (x_train1 > x_train2)
		{
			x_aux = x_train1;
			x_train1 = x_train2;
			x_train2 = x_aux;
		}

		if (y_train1 > y_train2)
		{
			y_aux = y_train1;
			y_train1 = y_train2;
			y_train2 = y_aux;
		}

	}

	printf("Button was clicked.\n");
	printf("X1 = %g; Y1 = %g; X2 = %g; Y2 = %g\n", x_train1, y_train1, x_train2, y_train2);
}

static void
start_graphics(int argc, char *argv[])
{
	GtkWidget *main_window;

	gtk_init(&argc, &argv);

	main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title (GTK_WINDOW (main_window), "Visual Search View");

	horizontal_container = gtk_hbox_new(TRUE, 1);

	right_image_drawing_area = gtk_drawing_area_new ();

	gtk_widget_set_size_request (right_image_drawing_area, camera_params.width, camera_params.height);

	gtk_box_pack_start(GTK_BOX(horizontal_container), right_image_drawing_area, TRUE, TRUE, 0);

	gtk_container_add(GTK_CONTAINER(main_window), horizontal_container);

	gtk_signal_connect(GTK_OBJECT(right_image_drawing_area), "expose_event",
 		     (GtkSignalFunc)expose_event, NULL);

	gtk_signal_connect(GTK_OBJECT(right_image_drawing_area), "button_press_event", 
 		     (GtkSignalFunc)button_was_clicked, NULL);
  
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

	carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
	gtk_widget_show(right_image_drawing_area);
	gtk_widget_show(horizontal_container);
	gtk_widget_show(main_window);

	gtk_main();
}

void
alloc_images_memory()
{
	size = 3 * camera_params.width * camera_params.height;

	right_image_data = (guchar *) calloc(size , sizeof(guchar));
  
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

	carmen_tracker_define_message_query_train();
	carmen_tracker_define_message_query_test();
	carmen_tracker_define_message_position();
  
	//Subscribes to the bumblebee messages
	if(camera != -1)
	{
	  	carmen_bumblebee_basic_subscribe_stereoimage(camera, &visual_search_bumblebee_message, (carmen_handler_t)bumblebee_image_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	carmen_stereo_subscribe(camera, NULL, (carmen_handler_t) carmen_stereo_disparity_handler, CARMEN_SUBSCRIBE_LATEST);

	signal(SIGINT, shutdown_visual_search_view);

	start_graphics(argc, argv);

	return 0;
}
