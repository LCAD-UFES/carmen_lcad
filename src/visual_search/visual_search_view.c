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

//Kinect width/height will be used for kinect sensor operation
#define KINECT_MAX_WIDTH 640
#define KINECT_MAX_HEIGHT 480

#include <carmen/carmen_graphics.h>	//must come first
#include <carmen/visual_search_interface.h>
#include <carmen/camera_interface.h>	//carmen camera interface
#include <carmen/kinect_interface.h>

#include <cairo.h>	//libcairo
#include <gtk/gtk.h>	//gtk interface

static GtkWidget *horizontal_container;

static GtkWidget *drawing_area_reference_image;
static GdkPixbuf *reference_image;

static GtkWidget *drawing_area_static_image;
static GdkPixbuf *static_image;

static int camera;
static int bumblebee_basic_width;
static int bumblebee_basic_height;

guchar *left_image_data = NULL;
guchar *right_image_data = NULL;
static cairo_t *drawing_area_static_image_cairo;

VISUAL_SEARCH_STATE	visual_search_state;
static int		x_img_point,y_img_point;

double current_image_timestamp;
static carmen_visual_search_message			visual_search_view_training_message;
static carmen_visual_search_state_change_message	visual_search_view_state_change_message;

static carmen_bumblebee_basic_stereoimage_message	visual_search_bumblebee_stereo_image;
static carmen_visual_search_output_message		visual_search_view_output_message;
static carmen_kinect_depth_message 			visual_search_view_kinect_depth_message;
static carmen_kinect_video_message			visual_search_view_kinect_video_message;

static carmen_visual_search_test_message		visual_search_testing_message;		//Bumblebee
static carmen_visual_search_kinect_test_message		visual_search_kinect_testing_message;	//Kinect

#define CROSS_LENGHT	10

static void 
redraw()
{
  gdk_draw_pixbuf(drawing_area_reference_image->window,
		  drawing_area_reference_image->style->fg_gc[GTK_WIDGET_STATE (drawing_area_reference_image)],
		  reference_image, 0, 0, 0, 0,
 		  drawing_area_reference_image->allocation.width,
 		  drawing_area_reference_image->allocation.height,
 		  GDK_RGB_DITHER_NONE, 0, 0);
 		  
  gdk_draw_pixbuf(drawing_area_static_image->window,
		  drawing_area_static_image->style->fg_gc[GTK_WIDGET_STATE (drawing_area_static_image)],
		  static_image, 0, 0, 0, 0,
 		  drawing_area_static_image->allocation.width,
 		  drawing_area_static_image->allocation.height,
 		  GDK_RGB_DITHER_NONE, 0, 0);
}
static void 
redraw_only_reference_image()
{
  gdk_draw_pixbuf(drawing_area_reference_image->window,
		  drawing_area_reference_image->style->fg_gc[GTK_WIDGET_STATE (drawing_area_reference_image)],
		  reference_image, 0, 0, 0, 0,
 		  drawing_area_reference_image->allocation.width,
 		  drawing_area_reference_image->allocation.height,
 		  GDK_RGB_DITHER_NONE, 0, 0);
}

// Draws a cross at the X-Y selected position
static void
draw_cross(GtkWidget *drawing, int x_img_point, int y_img_point)
{
	drawing_area_static_image_cairo = gdk_cairo_create(drawing->window);

	cairo_set_line_width(drawing_area_static_image_cairo,5);
	cairo_set_source_rgba(drawing_area_static_image_cairo, 255, 0, 0, 1);

	cairo_move_to(drawing_area_static_image_cairo, x_img_point, y_img_point - CROSS_LENGHT/2);
	cairo_line_to(drawing_area_static_image_cairo, x_img_point, y_img_point + CROSS_LENGHT/2);
	cairo_move_to(drawing_area_static_image_cairo, x_img_point - CROSS_LENGHT/2, y_img_point);
	cairo_line_to(drawing_area_static_image_cairo, x_img_point + CROSS_LENGHT/2, y_img_point);
	cairo_stroke(drawing_area_static_image_cairo);

	cairo_destroy(drawing_area_static_image_cairo);
}

static void
bumblebee_image_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	unsigned char *image_data; 
	int i,size;

	size = stereo_image->image_size;			//bumblebee_basic_width * bumblebee_basic_height;
	image_data = stereo_image->raw_left;			// Left Image as reference image
	current_image_timestamp = stereo_image->timestamp;	// Last Image timestamp must be saved
	
	g_object_unref(reference_image);	//Unreferencing the gtk object
	g_object_unref(static_image);
	
	if(visual_search_state == WAITING_FOR_TRAINNING)
	{
		for (i = 0; i < size; i++)			// 3*size -> total number of channels
			left_image_data[i] = (guchar) image_data[i];

		for (i = 0; i < size; i++)			// Static image zeroing
			right_image_data[i] = (guchar) 0;
	
		reference_image = gdk_pixbuf_new_from_data(left_image_data, GDK_COLORSPACE_RGB,
		     	   	   	   	   	   FALSE, 8,  bumblebee_basic_width, bumblebee_basic_height, bumblebee_basic_width * 3, NULL, NULL);
		     	   	   	   	   	   
		static_image = gdk_pixbuf_new_from_data(right_image_data, GDK_COLORSPACE_RGB,
		     	   	   	   	   	   FALSE, 8,  bumblebee_basic_width, bumblebee_basic_height, bumblebee_basic_width * 3, NULL, NULL);

		redraw();	// Redraw image buffers
	}
	else if(visual_search_state == RUNNING_NETWORK)
	{
		for (i = 0; i < size; i++)
			left_image_data[i] = (guchar) image_data[i];
		
		reference_image = gdk_pixbuf_new_from_data(left_image_data, GDK_COLORSPACE_RGB,
		     	   	   	   	   	   FALSE, 8,  bumblebee_basic_width, bumblebee_basic_height, bumblebee_basic_width * 3, NULL, NULL);
		     	   	   	   	   	   
		redraw_only_reference_image();		// Redraw image buffers (static image is kept static =P)

		if (visual_search_view_output_message.x_point != -1 && visual_search_view_output_message.y_point != -1)
			draw_cross(drawing_area_reference_image, visual_search_view_output_message.x_point, visual_search_view_output_message.y_point);

		/* Now builds the test message from Bumblebee/Kinect message */

		visual_search_testing_message.reference_image_size =  3*bumblebee_basic_width * bumblebee_basic_height;
		visual_search_testing_message.reference_image = image_data;
		visual_search_testing_message.timestamp = current_image_timestamp;	//same bb timestamp
		visual_search_testing_message.host = carmen_get_host();
		visual_search_testing_message.scale = 1.0;				//fixed internal scale parameter

		//bumblebee test publishing
		#ifdef VIEWER_PUBLISH_TEST
		IPC_RETURN_TYPE err = IPC_publishData(CARMEN_VISUAL_SEARCH_TEST_MESSAGE_NAME, &visual_search_testing_message);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_SEARCH_TEST_MESSAGE_NAME);
		#endif
	}
}

static void
kinect_video_handler(carmen_kinect_video_message *message)
{
	unsigned char *data; 
	int i,size;

	size = message->width*message->height;
	data = message->video;
	current_image_timestamp = message->timestamp;	// Last Image timestamp must be saved
	
	g_object_unref(reference_image);	//Unreferencing the gtk object
	g_object_unref(static_image);
	
	if(visual_search_state == WAITING_FOR_TRAINNING)
	{
		for (i = 0; i < 3*size; i++)			// 3*size -> total number of channels
			left_image_data[i] = (guchar) data[i];

		for (i = 0; i < 3*size; i++)			// Static image zeroing
			right_image_data[i] = (guchar) 0;
	
		reference_image = gdk_pixbuf_new_from_data(left_image_data, GDK_COLORSPACE_RGB,
		     	   	   	   	   	   FALSE, 8,  message->width, message->height, message->width * 3, NULL, NULL);
		     	   	   	   	   	   
		static_image = gdk_pixbuf_new_from_data(right_image_data, GDK_COLORSPACE_RGB,
		     	   	   	   	   	   FALSE, 8,  message->width, message->height, message->width * 3, NULL, NULL);

		redraw();	// Redraw image buffers
	}
	else if(visual_search_state == RUNNING_NETWORK)
	{
		for (i = 0; i < 3*size; i++)
			left_image_data[i] = (guchar) data[i];
		
		reference_image = gdk_pixbuf_new_from_data(left_image_data, GDK_COLORSPACE_RGB,
		     	   	   	   	   	   FALSE, 8,  message->width, message->height, message->width * 3, NULL, NULL);
		     	   	   	   	   	   
		redraw_only_reference_image();		// Redraw image buffers (static image is kept static =P)

		if (visual_search_view_output_message.x_point != -1 && visual_search_view_output_message.y_point != -1)
			draw_cross(drawing_area_reference_image, visual_search_view_output_message.x_point, visual_search_view_output_message.y_point);

		/* Now builds the test message from Kinect video message */

		visual_search_kinect_testing_message.video_data_size =  3*message->width*message->height;
		visual_search_kinect_testing_message.video_data = data;
	}
}

/* Depth handler agregates the kinect depth message into the testing message*/
static void
kinect_depth_handler(carmen_kinect_depth_message *message)
{
	//TODO: Should write 
	visual_search_kinect_testing_message.depth_data_size = message->width*message->height;
	visual_search_kinect_testing_message.depth_data = message->depth;
	visual_search_kinect_testing_message.timestamp = message->timestamp;	//same kinect depth timestamp
	visual_search_kinect_testing_message.host = carmen_get_host();
	
	//viewer test publishing ( Kinect messages will be published according to the depth message income)
	#ifdef VIEWER_PUBLISH_TEST
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_VISUAL_SEARCH_TEST_MESSAGE_NAME, &visual_search_testing_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_SEARCH_TEST_MESSAGE_NAME);
	#endif
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
  redraw();
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

static gboolean
button_press_eventA(GtkWidget *widget, GdkEventButton *event, gpointer user_data)
{
	IPC_RETURN_TYPE err = IPC_OK;
	int size,i;
	gdouble x, y;
	guint button;
	
	widget = widget;	// keeping the compiler happy
	user_data = user_data;	// keeping the compiler happy

	button = ((GdkEventButton*)event)->button;	// Left Button = 1 ; Right Button = 3
	x = ((GdkEventButton*)event)->x;
	y = ((GdkEventButton*)event)->y;
	x_img_point = (int)x;
	y_img_point = (int)y;
	printf("%d, %d , %d\n", (int)x, (int)y, (int)button);
	
	if(visual_search_state == WAITING_FOR_TRAINNING && button == 1)
	{
		size = bumblebee_basic_width * bumblebee_basic_height;
		
		for (i = 0; i < 3*size; i++)			// 3*size -> total number of channels
			right_image_data[i] = left_image_data[i];

		static_image = gdk_pixbuf_new_from_data(right_image_data, GDK_COLORSPACE_RGB,
		   	   	   	   	   	   FALSE, 8,  bumblebee_basic_width, bumblebee_basic_height, bumblebee_basic_width * 3, NULL, NULL);
		
		redraw();

		draw_cross(drawing_area_static_image, x_img_point, y_img_point);

		visual_search_state = TRAINNING_NETWORK;	//switches the network state
		
		// Builds up the trainning message
		visual_search_view_training_message.x_point = x_img_point; 
		visual_search_view_training_message.y_point = y_img_point;
		visual_search_view_training_message.reference_image_size = 3*size;
		visual_search_view_training_message.reference_image = left_image_data;
		//visual_search_view_training_message.reference_image = static_image_data;
		visual_search_view_training_message.timestamp = current_image_timestamp;
		visual_search_view_training_message.host = carmen_get_host();
		
		// Sends up the trainning message
		err = IPC_publishData(CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_NAME, &visual_search_view_training_message);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_NAME);
		visual_search_state = RUNNING_NETWORK;
	}
	
	return FALSE;
}

static gboolean
button_press_eventB(GtkWidget *widget, GdkEventButton *event, gpointer user_data)
{
	IPC_RETURN_TYPE err = IPC_OK;
	guint button;
	
	widget = widget;	// keeping the compiler happy
	user_data = user_data;	// keeping the compiler happy
	button = ((GdkEventButton*)event)->button;	// Left Button = 1 ; Right Button = 3
	
	if(visual_search_state == RUNNING_NETWORK && button == 3)
	{
		// Builds up the state change message and publishes it
		visual_search_view_state_change_message.host = carmen_get_host();
		visual_search_view_state_change_message.timestamp = current_image_timestamp;
		visual_search_view_state_change_message.state = WAITING_FOR_TRAINNING;
		err = IPC_publishData(CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_NAME, &visual_search_view_state_change_message);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_NAME);
		
		visual_search_state = WAITING_FOR_TRAINNING;	//Waiting for trainning	
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
  drawing_area_reference_image = gtk_drawing_area_new ();
  drawing_area_static_image = gtk_drawing_area_new ();

  gtk_widget_set_size_request (drawing_area_reference_image, bumblebee_basic_width, bumblebee_basic_height);
  gtk_widget_set_size_request (drawing_area_static_image, bumblebee_basic_width, bumblebee_basic_height);

  gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_reference_image, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_static_image, TRUE, TRUE, 0);

  gtk_container_add(GTK_CONTAINER(main_window), horizontal_container);

  gtk_signal_connect(GTK_OBJECT(drawing_area_static_image), "expose_event",
 		     (GtkSignalFunc)expose_event, NULL);
  gtk_signal_connect(GTK_OBJECT(drawing_area_reference_image), "expose_event",
 		     (GtkSignalFunc)expose_event, NULL);
  
  gtk_signal_connect(GTK_OBJECT(drawing_area_static_image), "button_press_event", 
 		     (GtkSignalFunc)button_press_eventB, NULL);
  gtk_signal_connect(GTK_OBJECT(drawing_area_reference_image), "button_press_event",
 		     (GtkSignalFunc)button_press_eventA, NULL);


  gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
 		     (GtkSignalFunc)key_press_event, NULL);
  gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
 		     (GtkSignalFunc)key_release_event, NULL);

  gtk_widget_add_events(drawing_area_static_image,  GDK_EXPOSURE_MASK
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
  gtk_widget_show(drawing_area_static_image);
  gtk_widget_show(drawing_area_reference_image);
  gtk_widget_show(horizontal_container);
  gtk_widget_show(main_window);

  visual_search_state = WAITING_FOR_TRAINNING;	//switches the state into Waiting for trainning state
  gtk_main();
}


int
read_parameters(int argc, char **argv)
{
  int num_items;

  char bumblebee_string[256];

  sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

  //Reading bumblebee and kinect parameters
  carmen_param_t param_list[] = {
    {bumblebee_string, "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
    {bumblebee_string, "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL}
  };
  
  //Must read kinect 

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  return 0;
}

void
alloc_images_memory(int camera)
{
  if (camera == -1)
  {
  	left_image_data = (guchar *) calloc(3 * KINECT_MAX_WIDTH * KINECT_MAX_HEIGHT , sizeof(guchar));
  	right_image_data = (guchar *) calloc(3 * KINECT_MAX_WIDTH * KINECT_MAX_HEIGHT , sizeof(guchar));
  }
  else
  {
  	left_image_data = (guchar *) calloc(3 * bumblebee_basic_width * bumblebee_basic_height , sizeof(guchar));
  	right_image_data = (guchar *) calloc(3 * bumblebee_basic_width * bumblebee_basic_height , sizeof(guchar));
  }
  
  carmen_test_alloc(left_image_data);
  carmen_test_alloc(right_image_data);
}

int
main(int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);

  carmen_param_check_version(argv[0]);

  if ((argc != 2) && (argc != 3))
       carmen_die("%s: Wrong number of parameters. Visual Search Viewer requires either 1 or 2 parameters and received %d parameter(s). \nUsage:\n %s <camera_number> or\n %s <camera_number[LR]> <camera_number[LR]>", argv[0], argc-1, argv[0], argv[0]);

  camera = atoi(argv[1]);

  read_parameters(argc, argv);

  visual_search_state = NON_STARTED;
  
  //Window parameters are set throug Kinect/Bumblebee parameters
  alloc_images_memory(camera);

  //Register to publish trainning and state change messages
  carmen_visual_search_define_message_test();		//the visual search viewer now should publish test messages
  carmen_visual_search_define_message_train();
  carmen_visual_search_define_message_state_change();
  
  //Subscribe to the output messages (for publishing the returned point)
  carmen_visual_search_subscribe_output(&visual_search_view_output_message, NULL, CARMEN_SUBSCRIBE_LATEST);
  
  //Subscribes to the bumblebee messages
  if(camera != -1)
  	carmen_bumblebee_basic_subscribe_stereoimage(camera, &visual_search_bumblebee_stereo_image, (carmen_handler_t)bumblebee_image_handler, CARMEN_SUBSCRIBE_LATEST);

  //Subscribes to the kinect messages
  carmen_kinect_subscribe_depth_message(0, &visual_search_view_kinect_depth_message, (carmen_handler_t) kinect_depth_handler, CARMEN_SUBSCRIBE_ALL);
  carmen_kinect_subscribe_video_message(0, &visual_search_view_kinect_video_message, (carmen_handler_t) kinect_video_handler, CARMEN_SUBSCRIBE_ALL);

  signal(SIGINT, shutdown_visual_search_view);

  start_graphics(argc, argv);

  return 0;
}
