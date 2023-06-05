/* Carmen includes */
#include <carmen/carmen_graphics.h>
#include <carmen/carmen.h>
#include <carmen/global.h>

/* Stereo includes */
#include <carmen/stereo_interface.h>
#include <carmen/stereo_util.h>

/* V-Disparity includes */
#include <carmen/v_disparity.h>

/* OpenCV includes */
#include<opencv/cv.h>
#include<opencv/highgui.h>

/* Image Utils includes */
#include "image_utils.h"

/* Machine Learning Road Finding */
#include "ml_road_finding.h"

/* Camera Information */
static stereo_util stereo_util_instance;
static v_disparity v_disparity_instance;

/* V-Disparity parameters */
static double v_disparity_slope_threshould;

static int *perceptual_field_mask;

static IplImage *disparity_map = NULL;
static IplImage *right_image = NULL;

static CvScalar *samples;
static CvPoint *points;
static IplImage *v_disparity_map = NULL;
static IplImage *road_profile_image = NULL;
static unsigned short int *v_disparity_data;

// Init Parameters
static int stereo_width;
static int stereo_height;
static int stereo_disparity;
static int number_of_gaussians = 5;
static ml_road_finding road_finding_instance;
static int camera;

// Image show
static GtkWidget *drawing_area;
static GdkPixbuf *road_image_buffer;

static void shutdown_road_finding_view(int x)
{
  if(x == SIGINT)
  {
    carmen_verbose("Disconnecting Road Finding View Service.\n");
    exit(1);
  }
}


static void
pixbuf_destroyed(guchar *pixels,
    gpointer data __attribute__ ((unused)))
{
  free(pixels);
}


static void
redraw_viewer()
{
  road_image_buffer = gdk_pixbuf_new_from_data((guchar*)right_image->imageData, GDK_COLORSPACE_RGB,
      FALSE, 8,  right_image->width,
      right_image->height, right_image->widthStep,
      pixbuf_destroyed, NULL);

	if (road_image_buffer)
	{
		gdk_draw_pixbuf(drawing_area->window,
				drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
				GDK_PIXBUF(road_image_buffer), 0, 0, 0, 0,
				stereo_width,
				drawing_area->allocation.height,
				GDK_RGB_DITHER_NONE, 0, 0);
	}
}


static gint
expose_event(GtkWidget *widget __attribute__ ((unused)),
    GdkEventExpose *event __attribute__ ((unused)))
{
  redraw_viewer();
  return 1;
}


static gint
key_press_event(GtkWidget *widget __attribute__ ((unused)),
    GdkEventKey *key)
{
  if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK))
    shutdown_road_finding_view(SIGINT);

  if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
  	shutdown_road_finding_view(SIGINT);

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
updateIPC(gpointer data __attribute__ ((unused)), gint source __attribute__ ((unused)), GdkInputCondition condition __attribute__ ((unused)))
{
  carmen_ipc_sleep(0.01);
  carmen_graphics_update_ipc_callbacks(updateIPC);
}


static void
disparity_map_handler(carmen_simple_stereo_disparity_message *message)
{
  // copy the reference image
  memcpy(right_image->imageData, message->reference_image, message->reference_image_size * sizeof (unsigned char));

  struct timeval begin;
  gettimeofday(&begin, NULL);

  // v-disparity analysis
  double camera_height, camera_pitch, horizon_line;
  compute_v_disparity_info(v_disparity_data, v_disparity_map, v_disparity_slope_threshould, message->disparity, &camera_height, &camera_pitch, &horizon_line, v_disparity_instance);
  int n_road_lines = draw_road_profile_lines_in_v_disparity(road_profile_image, v_disparity_map, v_disparity_slope_threshould, v_disparity_instance);

  // set the image mask to sky removal
  memset(perceptual_field_mask, 1, stereo_width * stereo_height * sizeof(int));
  memset(perceptual_field_mask, 0, stereo_width * round(horizon_line) * sizeof(int));

  // safe window and gaussian training
  if (n_road_lines)
  {
    int n_samples = get_road_pixels_list(right_image, points, samples, road_profile_image, message->disparity, 1, v_disparity_instance);
    rgb_gaussian *gaussian = get_gaussian_in_samples(samples, n_samples);
    add_gaussian(&road_finding_instance, gaussian);
#ifdef DEBUG
    print_gaussian(gaussian, stdout);
#endif
  }

  // filter the image
  fill_image_based_on_gaussians(&road_finding_instance, right_image, right_image, perceptual_field_mask);

  show_time((char*)"Image Classification", begin);

  redraw_viewer();
}


static void
start_graphics(int argc, char *argv[])
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (main_window), "Road Finding View");

  drawing_area = gtk_drawing_area_new ();
  gtk_widget_set_usize (drawing_area, stereo_width, stereo_height);

  gtk_container_add(GTK_CONTAINER(main_window), drawing_area);

  gtk_signal_connect(GTK_OBJECT(drawing_area), "expose_event",
  		(GtkSignalFunc)expose_event, NULL);
  gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
      (GtkSignalFunc)key_press_event, NULL);
  gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
      (GtkSignalFunc)key_release_event, NULL);

  gtk_widget_add_events(drawing_area,  GDK_EXPOSURE_MASK
      | GDK_BUTTON_PRESS_MASK
      | GDK_BUTTON_RELEASE_MASK
      | GDK_POINTER_MOTION_MASK
      | GDK_POINTER_MOTION_HINT_MASK
      | GDK_KEY_PRESS_MASK
      | GDK_KEY_RELEASE_MASK);

  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  gtk_widget_show(drawing_area);
  gtk_widget_show(main_window);

  gtk_main();
}


static int
read_parameters(int argc, char **argv)
{
  int num_items;
  char stereo_string[256];

  if (argc < 2)
    carmen_die("%s: Wrong number of parameters.road finding view requires a parameter.\nUsage:\n %s <camera_number>\n", argv[0], argv[0]);

  camera = atoi(argv[1]);
  sprintf(stereo_string, "%s%d", "stereo", camera);

  carmen_param_t param_list[] = {
      {stereo_string, (char*)"width", CARMEN_PARAM_INT, &stereo_width, 0, NULL},
      {stereo_string, (char*)"height", CARMEN_PARAM_INT, &stereo_height, 0, NULL},
      {stereo_string, (char*)"max_disparity", CARMEN_PARAM_INT, &stereo_disparity, 0, NULL},
      {(char*)"road_finding", (char*)"number_of_gaussians", CARMEN_PARAM_INT, &number_of_gaussians, 5, NULL},
      {(char*) "v_disparity", (char*) "slope_threshould", CARMEN_PARAM_DOUBLE, &v_disparity_slope_threshould, 0, NULL}
  };

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  return 0;
}


static void
init_road_finding_view()
{
  if (disparity_map == NULL)
  {
    disparity_map = cvCreateImage(cvSize(stereo_width, stereo_height), IPL_DEPTH_8U, 3);
  }

  if (right_image == NULL)
  {
    right_image = cvCreateImage(cvSize(stereo_width, stereo_height), IPL_DEPTH_8U, 3);
  }

  // create ml_road_finding
  init_ml_road_finding(&road_finding_instance, number_of_gaussians, 3 * stereo_width, stereo_height);

  /* Create a stereo_util instance and a v-disparity util instance*/
  stereo_util_instance = get_stereo_instance(camera, stereo_width, stereo_height);
  v_disparity_instance = get_v_disparity_instance(stereo_util_instance, stereo_disparity);

  if (samples == NULL)
  {
    samples = (CvScalar*)malloc(stereo_width * stereo_height * sizeof(CvScalar));
  }

  if (points == NULL)
  {
    points = (CvPoint*)malloc(stereo_width * stereo_height * sizeof(CvPoint));
  }

  if (v_disparity_map == NULL)
  {
    v_disparity_map = cvCreateImage(cvSize(stereo_disparity, stereo_height), IPL_DEPTH_8U, 1);
  }

  if (road_profile_image == NULL)
  {
    road_profile_image = cvCreateImage(cvGetSize(v_disparity_map), IPL_DEPTH_8U, 3);
  }

  if (v_disparity_data == NULL)
  {
    v_disparity_data = alloc_v_disparity_map(v_disparity_instance);
  }

  /* Init right image mask */
  perceptual_field_mask = (int*)malloc(stereo_width * stereo_height * sizeof(int));
}

int
main(int argc, char **argv)
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_road_finding_view);

  /* Initialize all the relevant parameters */
  read_parameters(argc, argv);

  /* Allocate my own structures */
  init_road_finding_view();

  /* Subscribe to Stereo Services */
  carmen_stereo_subscribe(camera, NULL, (carmen_handler_t) disparity_map_handler, CARMEN_SUBSCRIBE_LATEST);

  /* Start the graphics windows */
  start_graphics(argc, argv);

  return 0;
}
