#include <carmen/carmen_graphics.h>
#include <carmen/xsens_interface.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/visual_odometry_interface.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#define TEST_GPS_LATENCY
#define TEST_GPS_LATENCY_USING_POSITION

#define	GRAPH_WIDTH		1200
#define	GRAPH_HEIGHT		600
#define	QUEUE_SIZE		(GRAPH_WIDTH * 4)

#ifdef	TEST_GPS_LATENCY
#ifdef 	TEST_GPS_LATENCY_USING_POSITION
#define TIME_WINDOW		20.0
#define GAIN			10.0
#define	BIAS			(GRAPH_HEIGHT / 3)
#else
#define TIME_WINDOW		20.0
#define GAIN			30.0
#define	BIAS			0.0
#endif
#else
#define TIME_WINDOW		5.0
#define GAIN			300.0
#define	BIAS			0.0
#endif

static carmen_xsens_global_matrix_message 	xsens_matrix_message, xsens_matrix_message_queue[QUEUE_SIZE];
static carmen_laser_laser_message 		laser_message, laser_message_queue[QUEUE_SIZE];
static carmen_visual_odometry_pose6d_message 	visual_odometry_message, visual_odometry_message_queue[QUEUE_SIZE];
static carmen_gps_xyz_message 			gps_message, gps_message_queue[QUEUE_SIZE];

static int xsens_num_messages = 0;
static int laser_num_messages = 0;
static int visual_odometry_num_messages = 0;
static int gps_num_messages = 0;

static int g_num_latency_graph_points = 100;

static GtkWidget *horizontal_container;
static GtkWidget *drawing_area_left;
static GdkColor color_black;
static GdkColor color_red;
static GdkColor color_green;
static GdkColor color_blue;
static GdkGC *gc;

double xsens_initial_yaw = 0.0;
carmen_vector_3D_t gps_initial_pos = {0.0, 0.0, 0.0};

double camera_to_xsens_latency = 0.0;
double camera_to_gps_latency = 0.5;


/*
static double 
get_roll_from_xsens_matrix_message(carmen_xsens_global_matrix_message xsens_matrix_message)
{
	return(atan2(xsens_matrix_message.matrix_data.m_data[2][1], xsens_matrix_message.matrix_data.m_data[2][2]));
}


static double 
get_pitch_from_xsens_matrix_message(carmen_xsens_global_matrix_message xsens_matrix_message)
{
	return(-asin(xsens_matrix_message.matrix_data.m_data[2][0]));
}
*/


static double 
get_yaw_from_xsens_matrix_message(carmen_xsens_global_matrix_message xsens_matrix_message)
{
	return(atan2(xsens_matrix_message.matrix_data.m_data[1][0], xsens_matrix_message.matrix_data.m_data[0][0]));
}


void 
draw_camera_to_xsens_graph(void)
{
	int x, visual_odometry_index, xsens_index;
	double time_displacement;
	
	gdk_window_clear(drawing_area_left->window);

	visual_odometry_index = 1;
	for (x = 0; x < GRAPH_WIDTH - 2; x++)
	{
		time_displacement = TIME_WINDOW * ((double) x / (double) GRAPH_WIDTH);
		if (visual_odometry_index < visual_odometry_num_messages)
		{	
			while (((visual_odometry_message_queue[0].timestamp - visual_odometry_message_queue[visual_odometry_index].timestamp) - camera_to_xsens_latency) < time_displacement)
				visual_odometry_index++;

			gdk_gc_set_rgb_fg_color(gc, &color_green);
			gdk_draw_rectangle(drawing_area_left->window, gc, TRUE, 
				x, BIAS + GRAPH_HEIGHT / 2 - GAIN * (visual_odometry_message_queue[visual_odometry_index - 1].pose_6d.yaw - xsens_initial_yaw),
				2, 2);
		}
	}

	xsens_index = 1;
	for (x = 0; x < GRAPH_WIDTH - 2; x++)
	{
		time_displacement = TIME_WINDOW * ((double) x / (double) GRAPH_WIDTH);
		if (xsens_index < xsens_num_messages)
		{
			while ((visual_odometry_message_queue[0].timestamp - xsens_matrix_message_queue[xsens_index].timestamp) < time_displacement)
				xsens_index++;

			gdk_gc_set_rgb_fg_color(gc, &color_red);
			gdk_draw_rectangle(drawing_area_left->window, gc, TRUE, 
				x, BIAS + GRAPH_HEIGHT / 2 - GAIN * (get_yaw_from_xsens_matrix_message(xsens_matrix_message_queue[xsens_index - 1]) - xsens_initial_yaw),
				2, 2);
		}
	}
}


double
visual_odometry_velocity(int index)
{
	double delta_t;
	double delta_x;
	double velocity;
	
	delta_t = visual_odometry_message_queue[index].timestamp - visual_odometry_message_queue[index - 1].timestamp;
	delta_x = visual_odometry_message_queue[index].pose_6d.x - visual_odometry_message_queue[index - 1].pose_6d.x;
	velocity = delta_x / delta_t;
	return (velocity);
}


double
gps_velocity(int index)
{
	double delta_t;
	double delta_x;
	double velocity;
	
	delta_t = gps_message_queue[index].timestamp - gps_message_queue[index - 1].timestamp;
	delta_x = gps_message_queue[index].x - gps_message_queue[index - 1].x;
	velocity = delta_x / delta_t;
	return (velocity);
}


void 
draw_camera_to_gps_graph(void)
{
	int x, visual_odometry_index, gps_index;
	double time_displacement;
	
	gdk_window_clear(drawing_area_left->window);

	visual_odometry_index = 1;
	for (x = 0; x < GRAPH_WIDTH - 2; x++)
	{
		time_displacement = TIME_WINDOW * ((double) x / (double) GRAPH_WIDTH);
		if (visual_odometry_index < visual_odometry_num_messages)
		{	
			while (((visual_odometry_message_queue[0].timestamp - visual_odometry_message_queue[visual_odometry_index].timestamp) - camera_to_gps_latency) < time_displacement)
				visual_odometry_index++;

			gdk_gc_set_rgb_fg_color(gc, &color_green);
#ifdef TEST_GPS_LATENCY_USING_POSITION
			gdk_draw_rectangle(drawing_area_left->window, gc, TRUE, 
				x, BIAS + GRAPH_HEIGHT / 2 - GAIN * visual_odometry_message_queue[visual_odometry_index - 1].pose_6d.x,
				2, 2);
#else
			gdk_draw_rectangle(drawing_area_left->window, gc, TRUE, 
				x, BIAS + GRAPH_HEIGHT / 2 - GAIN * visual_odometry_velocity(visual_odometry_index),
				2, 2);
#endif
		}
	}

	gps_index = 1;
	for (x = 0; x < GRAPH_WIDTH - 2; x++)
	{
		time_displacement = TIME_WINDOW * ((double) x / (double) GRAPH_WIDTH);
		if (gps_index < gps_num_messages)
		{
			while ((visual_odometry_message_queue[0].timestamp - gps_message_queue[gps_index].timestamp) < time_displacement)
				gps_index++;

			gdk_gc_set_rgb_fg_color(gc, &color_red);
#ifdef TEST_GPS_LATENCY_USING_POSITION
			gdk_draw_rectangle(drawing_area_left->window, gc, TRUE, 
				x, BIAS + GRAPH_HEIGHT / 2 - GAIN * gps_message_queue[gps_index - 1].x,
				2, 2);
#else
			gdk_draw_rectangle(drawing_area_left->window, gc, TRUE, 
				x, BIAS + GRAPH_HEIGHT / 2 - GAIN * gps_velocity(gps_index),
				2, 2);
#endif
		}
	}
}


static void 
redraw(void)
{
#ifdef	TEST_GPS_LATENCY
	draw_camera_to_gps_graph();
#else
	draw_camera_to_xsens_graph();
#endif
}


static void
add_message_to_xsens_message_queue()
{
	int i;
	
	for (i = QUEUE_SIZE - 1; i > 0; i--)
	{
		xsens_matrix_message_queue[i] = xsens_matrix_message_queue[i-1];
	}
	xsens_matrix_message_queue[0] = xsens_matrix_message;
}


static void
add_message_to_laser_message_queue()
{
	int i;
	
	for (i = QUEUE_SIZE - 1; i > 0; i--)
	{
		laser_message_queue[i] = laser_message_queue[i-1];
	}
	laser_message_queue[0] = laser_message;
}


static void
add_message_to_visual_odometry_message_queue()
{
	int i;
	
	for (i = QUEUE_SIZE - 1; i > 0; i--)
	{
		visual_odometry_message_queue[i] = visual_odometry_message_queue[i-1];
	}
	visual_odometry_message_queue[0] = visual_odometry_message;
}


static void
add_message_to_gps_message_queue()
{
	int i;
	
	for (i = QUEUE_SIZE - 1; i > 0; i--)
	{
		gps_message_queue[i] = gps_message_queue[i-1];
	}
	gps_message_queue[0] = gps_message;
}


static void 
xsens_matrix_message_handler(void) 
{
	xsens_num_messages++;
	add_message_to_xsens_message_queue();
	//printf("xsens_num_messages = %d\n", xsens_num_messages);

	redraw();
}


static void 
laser_message_handler(void)
{	
	laser_num_messages++;
	add_message_to_laser_message_queue();
	//printf("laser_num_messages = %d\n", laser_num_messages);
}


static void 
visual_odometry_message_handler(void)
{
	double x, z;
	
	if (gps_num_messages == 10)
		xsens_initial_yaw = get_yaw_from_xsens_matrix_message(xsens_matrix_message_queue[0]);
		
	x = visual_odometry_message.pose_6d.x;
	z = visual_odometry_message.pose_6d.z;
	visual_odometry_message.pose_6d.x = x * cos(xsens_initial_yaw) - z * sin(xsens_initial_yaw);
	visual_odometry_message.pose_6d.z = x * sin(xsens_initial_yaw) + z * cos(xsens_initial_yaw);
	visual_odometry_message.pose_6d.yaw = carmen_normalize_theta(visual_odometry_message.pose_6d.yaw + xsens_initial_yaw);

	visual_odometry_num_messages++;
	add_message_to_visual_odometry_message_queue();
	//printf("visual_odometry_num_messages = %d\n", visual_odometry_num_messages);

	redraw();
}


static void 
gps_message_handler(void)
{	
	if (gps_num_messages == 10)
	{
		gps_initial_pos.x = gps_message.x;
		gps_initial_pos.y = gps_message.y;
		gps_initial_pos.z = gps_message.z;
	}
	gps_message.y = gps_message.y - gps_initial_pos.y;
	gps_message.x = gps_message.x - gps_initial_pos.x;
	gps_message.z = gps_message.z - gps_initial_pos.z;
	
	gps_num_messages++;
	add_message_to_gps_message_queue();
	//printf("gps_num_messages = %d\n", gps_num_messages);
}


static gint
updateIPC(gpointer *data __attribute__ ((unused)))
{
  carmen_ipc_sleep(0.01);
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  return 1;
}


static gint
expose_event(GtkWidget *widget __attribute__ ((unused)),
	     GdkEventExpose *event __attribute__ ((unused)))
{
  redraw();
  return 1;
}


static gint
key_press_event(GtkWidget *widget __attribute__ ((unused)),
		GdkEventKey *key)
{
  redraw();
  if (key->state || key->keyval > 255)
    return 1;

  return 1;
}


static gint
key_release_event(GtkWidget *widget __attribute__ ((unused)),
		  GdkEventButton *key __attribute__ ((unused)))
{
  redraw();
  return 1;
}


static void
start_graphics(int argc, char *argv[])
{
  GtkWidget *main_window;
  char window_name[100];

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  sprintf(window_name, "Latency Estimation Module");
  gtk_window_set_title (GTK_WINDOW (main_window), window_name);

  horizontal_container = gtk_hbox_new(TRUE, 1);
  drawing_area_left = gtk_drawing_area_new ();

  gtk_widget_set_usize (drawing_area_left, GRAPH_WIDTH, GRAPH_HEIGHT);
  gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_left, TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(main_window), horizontal_container);

  gtk_signal_connect(GTK_OBJECT(drawing_area_left), "expose_event",
		     (GtkSignalFunc)expose_event, NULL);

  gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
		     (GtkSignalFunc)key_press_event, NULL);
  gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
		     (GtkSignalFunc)key_release_event, NULL);

  gtk_widget_add_events(drawing_area_left,  GDK_EXPOSURE_MASK
			| GDK_BUTTON_PRESS_MASK
			| GDK_BUTTON_RELEASE_MASK
			| GDK_POINTER_MOTION_MASK
			| GDK_POINTER_MOTION_HINT_MASK
			| GDK_KEY_PRESS_MASK
			| GDK_KEY_RELEASE_MASK);

  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  gtk_widget_show(drawing_area_left);
  gtk_widget_show(horizontal_container);
  gtk_widget_show(main_window);

  gdk_drawable_set_colormap(drawing_area_left->window, gdk_rgb_get_colormap());
  gc = gdk_gc_new(drawing_area_left->window);

  color_black.red = 0;
  color_black.green = 0;
  color_black.blue = 0;

  color_red.red = 65535;
  color_red.green = 0;
  color_red.blue = 0;

  color_green.red = 0;
  color_green.green = 65535;
  color_green.blue = 0;

  color_blue.pixel = 3;
  color_blue.red = 0;
  color_blue.green = 0;
  color_blue.blue = 65535;

  gtk_main();
}



int 
main(int argc, char** argv)
{ 
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	if (argc > 1)
		g_num_latency_graph_points = atoi(argv[1]);

	/* Subscribe to sensor messages */
	carmen_xsens_subscribe_xsens_global_matrix_message(&xsens_matrix_message,
		                                	   (carmen_handler_t)xsens_matrix_message_handler,
		                                	   CARMEN_SUBSCRIBE_LATEST);

	carmen_laser_subscribe_frontlaser_message(&laser_message,
				   		  (carmen_handler_t)laser_message_handler,
				     		  CARMEN_SUBSCRIBE_LATEST);

	carmen_visual_odometry_subscribe_pose6d_message(&visual_odometry_message,
			      				(carmen_handler_t)visual_odometry_message_handler, 
							CARMEN_SUBSCRIBE_LATEST);

	carmen_gps_xyz_subscribe_message(&gps_message,
		                	 (carmen_handler_t)gps_message_handler,
		                	 CARMEN_SUBSCRIBE_LATEST);	

	/* Start graph and loop forever waiting for messages */
	start_graphics(argc, argv);

	return (0);
}
