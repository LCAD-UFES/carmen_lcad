/* Carmen includes */
#include <carmen/carmen_graphics.h>
#include <carmen/carmen.h>
#include <carmen/global.h>

/* My own includes */
#include "v_disparity.h"

/* Stereo includes */
#include <carmen/stereo_util.h>

/* OpenCV Includes */
#include <opencv/cv.h>
#include <opencv/highgui.h>

/* Image show */
static GtkWidget *drawing_area;
static GdkPixbuf *v_disparity_buffer;
static GdkPixbuf *road_profile_buffer;
static GdkPixbuf *right_buffer = NULL;

static IplImage *right_image = NULL;
static IplImage *v_disparity_map = NULL;
static IplImage *v_disparity_map_3channels = NULL;
static IplImage *road_profile_image = NULL;

static CvScalar *samples;
static CvPoint *points;

static unsigned short int *v_disparity_data;

/* Parameters */
static int stereo_width;
static int stereo_height;
static int stereo_disparity;
static double v_disparity_slope_threshould;

/* Camera Index */
static int camera;

/* Camera Information */
static stereo_util stereo_util_instance;
static v_disparity v_disparity_instance;

static void
shutdown_v_disparitity_view(int x)
{
  /* release memory */
  cvReleaseImage(&right_image);
  cvReleaseImage(&v_disparity_map);
  cvReleaseImage(&road_profile_image);

  /* exit module */
  if(x == SIGINT)
  {
    carmen_verbose("Disconnecting V-Disparity View Service.\n");
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
flush_gdk_buffers(void)
{
  v_disparity_buffer = gdk_pixbuf_new_from_data((guchar*)v_disparity_map_3channels->imageData, GDK_COLORSPACE_RGB,
      FALSE, 8,  v_disparity_map_3channels->width,
      v_disparity_map_3channels->height, v_disparity_map_3channels->widthStep,
      pixbuf_destroyed, NULL);

  road_profile_buffer = gdk_pixbuf_new_from_data((guchar*)road_profile_image->imageData, GDK_COLORSPACE_RGB,
      FALSE, 8,  road_profile_image->width,
      road_profile_image->height, road_profile_image->widthStep,
      pixbuf_destroyed, NULL);

  right_buffer = gdk_pixbuf_new_from_data((guchar*)right_image->imageData, GDK_COLORSPACE_RGB,
      FALSE, 8,  right_image->width,
      right_image->height, right_image->widthStep,
      pixbuf_destroyed, NULL);
}

static void
redraw_viewer(void)
{
  flush_gdk_buffers();

  if (v_disparity_buffer)
  {
    gdk_draw_pixbuf(drawing_area->window,
        drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
        GDK_PIXBUF(v_disparity_buffer), 0, 0, 0, 0,
        v_disparity_map->width,
        drawing_area->allocation.height,
        GDK_RGB_DITHER_NONE, 0, 0);
  }

  if (road_profile_buffer)
  {
    gdk_draw_pixbuf(drawing_area->window,
        drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
        GDK_PIXBUF(road_profile_buffer), 0, 0, v_disparity_map->width + 10, 0,
        road_profile_image->width,
        drawing_area->allocation.height,
        GDK_RGB_DITHER_NONE, 0, 0);
  }

  if (right_buffer)
  {
    gdk_draw_pixbuf(drawing_area->window,
        drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
        GDK_PIXBUF(right_buffer), 0, 0,  v_disparity_map->width + road_profile_image->width + 20, 0,
        right_image->width,
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
    shutdown_v_disparitity_view(SIGINT);

  if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
    shutdown_v_disparitity_view(SIGINT);

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
  memcpy(right_image->imageData, message->reference_image, message->reference_image_size);

  // v-disparity analysis
  double camera_height, camera_pitch, horizon_line;
  compute_v_disparity_info(v_disparity_data, v_disparity_map, v_disparity_slope_threshould, message->disparity, &camera_height, &camera_pitch, &horizon_line, v_disparity_instance);
  draw_road_profile_lines_in_v_disparity(road_profile_image, v_disparity_map, v_disparity_slope_threshould, v_disparity_instance);

  // merge the one channel v-disparity image into a 3-channel v-disparity image
  cvMerge(v_disparity_map, v_disparity_map, v_disparity_map, NULL, v_disparity_map_3channels);

  // print all the obstacles to the camera image
  print_obstacles(right_image, v_disparity_map, road_profile_image, message->disparity, camera_height, camera_pitch, v_disparity_instance);

  // fill the ground plane
  int n_samples = get_road_pixels_list(right_image, points, samples, road_profile_image, message->disparity, 0, v_disparity_instance);
  int i;
  for (i = 0; i < n_samples; i++)
  {
    CvScalar pixel = cvGet2D(right_image, points[i].y, points[i].x);
    pixel.val[1] = 255;
    cvSet2D(right_image, points[i].y, points[i].x, pixel);
  }

  // draw a blue line to represent the horizon line
  if (horizon_line >=0 && horizon_line < right_image->height)
  {
    CvPoint h_start = cvPoint(0, horizon_line);
    CvPoint h_end = cvPoint(right_image->width - 1, horizon_line);
    cvLine(right_image, h_start, h_end, CV_RGB(255,0,0), 1, 4, 0);
  }

  redraw_viewer();
}

static void
start_graphics(int argc, char *argv[])
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

  gtk_window_set_title (GTK_WINDOW (main_window), "V-Disparity Robot View");
  drawing_area = gtk_drawing_area_new ();

  gtk_widget_set_usize (drawing_area, stereo_disparity + stereo_disparity + stereo_width + 20, stereo_height);

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
    carmen_die("%s: Wrong number of parameters. v-disparity view requires a parameter.\nUsage:\n %s <camera_number>\n", argv[0], argv[0]);

  camera = atoi(argv[1]);
  sprintf(stereo_string, "%s%d", "stereo", camera);

  carmen_param_t param_list[] = {
      {stereo_string, (char*) "width", CARMEN_PARAM_INT, &stereo_width, 0, NULL},
      {stereo_string, (char*) "height", CARMEN_PARAM_INT, &stereo_height, 0, NULL},
      {stereo_string, (char*) "max_disparity", CARMEN_PARAM_INT, &stereo_disparity, 0, NULL},
      {(char*) "v_disparity", (char*) "slope_threshould", CARMEN_PARAM_DOUBLE, &v_disparity_slope_threshould, 0, NULL}
  };

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  /* Create a stereo_util instance and a v-disparity util instance*/
  stereo_util_instance = get_stereo_instance(camera, stereo_width, stereo_height);
  v_disparity_instance = get_v_disparity_instance(stereo_util_instance, stereo_disparity);

  return 0;
}


static void
init_v_disparity_viewer()
{
  if (right_image == NULL)
  {
    right_image = cvCreateImage(cvSize(stereo_width, stereo_height), IPL_DEPTH_8U, 3);
  }

  if (v_disparity_map == NULL)
  {
    v_disparity_map = cvCreateImage(cvSize(stereo_disparity, stereo_height), IPL_DEPTH_8U, 1);
  }

  if (v_disparity_map_3channels == NULL)
  {
    v_disparity_map_3channels = cvCreateImage(cvSize(stereo_disparity, stereo_height), IPL_DEPTH_8U, 3);
  }

  if (road_profile_image == NULL)
  {
    road_profile_image = cvCreateImage(cvGetSize(v_disparity_map), IPL_DEPTH_8U, 3);
  }

  if (v_disparity_data == NULL)
  {
    v_disparity_data = alloc_v_disparity_map(v_disparity_instance);
  }

  if (samples == NULL)
  {
    samples = (CvScalar*)malloc(stereo_width * stereo_height * sizeof(CvScalar));
  }

  if (points == NULL)
  {
    points = (CvPoint*)malloc(stereo_width * stereo_height * sizeof(CvPoint));
  }
}


int
main(int argc, char **argv)
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_v_disparitity_view);

  /* Initialize all the relevant parameters */
  read_parameters(argc, argv);

  /* Allocate my own structures */
  init_v_disparity_viewer();

  /* Subscribe to Stereo Services */
  carmen_stereo_subscribe(camera, NULL, (carmen_handler_t) disparity_map_handler, CARMEN_SUBSCRIBE_LATEST);

  /* Start the graphics windows */
  start_graphics(argc, argv);

  return 0;
}
