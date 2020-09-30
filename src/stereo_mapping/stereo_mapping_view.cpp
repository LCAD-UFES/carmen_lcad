/* Carmen includes */
#include <carmen/carmen_graphics.h>
#include <carmen/carmen.h>
#include <carmen/global.h>

/* My own includes */
#include "stereo_mapping_interface.h"

/* Stereo includes */
#include <carmen/stereo_util.h>

/* OpenCV Includes */
#include <opencv/cv.h>
#include <opencv/highgui.h>

/* Prob Models includes */
#include <prob_measurement_model.h>
#include <prob_map.h>

/* Image show */
static GtkWidget *drawing_area;
static GdkPixbuf *stereo_mapping_buffer;

/* Stereo Map Image */
static double stereo_map_timestamp;
static IplImage *stereo_map;

/* Init Parameters */
static int camera;

/* View Mode */
static int view_mode = 0;

/* Map Parameters */
static ProbabilisticMapParams map_params;

static void
shutdown_stereo_mapping_view(int x)
{
  /* release memory */
  cvReleaseImage(&stereo_map);

  /* exit module */
  if(x == SIGINT)
  {
    carmen_verbose("Disconnecting Stereo Mapping View Service.\n");
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
redraw_viewer(void)
{
  if (stereo_mapping_buffer)
  {
    gdk_draw_pixbuf(drawing_area->window,
        drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
        GDK_PIXBUF(stereo_mapping_buffer), 0, 0, 0, 0,
        drawing_area->allocation.width,
        drawing_area->allocation.height,
        GDK_RGB_DITHER_NONE, 0, 0);
  }
}


static void
stereo_mapping_handler(carmen_stereo_mapping_message *message)
{
  // copy the image
  stereo_map_timestamp = message->timestamp;
  memcpy(stereo_map->imageData, message->stereo_mapping_data, message->map_size);
  cvCvtColor(stereo_map, stereo_map, CV_BGR2RGB);

  // Build the pixel buffer
  stereo_mapping_buffer = gdk_pixbuf_new_from_data((guchar*)message->stereo_mapping_data, GDK_COLORSPACE_RGB,
      FALSE, 8, map_params.grid_sx,
      map_params.grid_sy, 3 * map_params.grid_sx,
      pixbuf_destroyed, NULL);

  redraw_viewer();
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
    shutdown_stereo_mapping_view(SIGINT);

  if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
    shutdown_stereo_mapping_view(SIGINT);

  // save the image on disk
  if (toupper(key->keyval) == 'S' && (key->state & GDK_CONTROL_MASK))
  {
    char image_name[1024];
    sprintf(image_name, "stereo_map_%f.bmp", stereo_map_timestamp);
    cvSaveImage(image_name, stereo_map);
  }

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


static gint
updateIPC(gpointer *data __attribute__ ((unused)))
{
  carmen_ipc_sleep(0.1);
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  return 1;
}


static void
start_graphics(int argc, char *argv[])
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (main_window), "Stereo Mapping View");

  drawing_area = gtk_drawing_area_new ();
  gtk_widget_set_usize (drawing_area, map_params.grid_sx, map_params.grid_sy);

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

  if (argc < 2)
    carmen_die("%s: Wrong number of parameters. v-disparity requires a parameter.\nUsage:\n %s <camera_number>\n", argv[0], argv[0]);

  camera = atoi(argv[1]);

  carmen_param_t param_list[] = {
      {(char*)"stereo_map", (char*)"width", CARMEN_PARAM_DOUBLE, &map_params.width, 0, NULL},
      {(char*)"stereo_map", (char*)"height", CARMEN_PARAM_DOUBLE, &map_params.height, 0, NULL},
      {(char*)"stereo_map", (char*)"grid_res", CARMEN_PARAM_DOUBLE, &map_params.grid_res, 0, NULL}
  };

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  map_params.grid_sx = (int)ceil(map_params.width / map_params.grid_res);
  map_params.grid_sy = (int)ceil(map_params.height / map_params.grid_res);

  stereo_map = cvCreateImage(cvSize(map_params.grid_sx, map_params.grid_sy), IPL_DEPTH_8U, 3);

  return 0;
}


static void init_stereo_mapping_viewer()
{
  view_mode = VIEW_MODE_BIRDS_EYE;
}


int
main(int argc, char **argv)
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_stereo_mapping_view);

  /* Initialize all the relevant parameters */
  read_parameters(argc, argv);

  /* Allocate my own structures */
  init_stereo_mapping_viewer();

  /* Subscribe to Stereo Mapping Service */
  carmen_stereo_mapping_subscribe_message(NULL,
          (carmen_handler_t) stereo_mapping_handler,
          CARMEN_SUBSCRIBE_LATEST,
          camera);

  /* Start the graphics windows */
  start_graphics(argc, argv);

  return 0;
}
