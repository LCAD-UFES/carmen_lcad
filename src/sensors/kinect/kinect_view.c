#include <carmen/carmen_graphics.h>
#include <carmen/camera_interface.h>
#include <carmen/kinect_interface.h>
#include "kinect_util.h"

#define MAX_WIDTH 640
#define MAX_HEIGHT 480

static GtkWidget *drawing_area;
static GdkPixbuf *video;
static GdkPixbuf *depth;
static int received_video = 0;
static int received_depth = 0;
static void redraw(void);
static unsigned short int m_gamma[2048];//2^11
static unsigned char m_buffer_depth[MAX_WIDTH*MAX_HEIGHT*3];


static void
init()
{
  unsigned int i;
  for(i = 0 ; i < 2048 ; i++) {
    float v = i/2048.0;
    v = pow(v, 3)* 6;
    m_gamma[i] = v*6*256;
  }
}

static void
pixbuf_destroyed(guchar *pixels,
    gpointer data __attribute__ ((unused)))
{
  free(pixels);
}

static void
convertDepthFromMetersToRGB(float* depth_in_meters, int width, int height){
	  int i;
	  uint16_t depth;
	  for(i = 0 ; i < height*width ; i++) {
		  depth = convert_kinect_depth_meters_to_raw(depth_in_meters[i]);

			int pval = m_gamma[depth];
			int lb = pval & 0xff;
			switch (pval>>8) {
			case 0:
				m_buffer_depth[3*i+0] = 255;
				m_buffer_depth[3*i+1] = 255-lb;
				m_buffer_depth[3*i+2] = 255-lb;
				break;
			case 1:
				m_buffer_depth[3*i+0] = 255;
				m_buffer_depth[3*i+1] = lb;
				m_buffer_depth[3*i+2] = 0;
				break;
			case 2:
				m_buffer_depth[3*i+0] = 255-lb;
				m_buffer_depth[3*i+1] = 255;
				m_buffer_depth[3*i+2] = 0;
				break;
			case 3:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 255;
				m_buffer_depth[3*i+2] = lb;
				break;
			case 4:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 255-lb;
				m_buffer_depth[3*i+2] = 255;
				break;
			case 5:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 0;
				m_buffer_depth[3*i+2] = 255-lb;
				break;
			default:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 0;
				m_buffer_depth[3*i+2] = 0;
				break;
			}
	  }
}

static void
ipc_kinect_depth_handler(carmen_kinect_depth_message *message)
{
  int i, j;
  unsigned char *data;

  if (!received_depth) {
    gtk_widget_set_usize (drawing_area, message->width*2, message->height);
  } else
    g_object_unref(depth);

  convertDepthFromMetersToRGB(message->depth, message->width, message->height);

  data = (unsigned char *)calloc(message->width*message->height*3, sizeof(unsigned char));
  carmen_test_alloc(data);

  for (i = 0; i < message->width*message->height; i++) {
    for (j = 0; j < 3; j++) {
      data[3*i+j] = m_buffer_depth[3*i+j];
    }
  }

  depth = gdk_pixbuf_new_from_data((guchar *)data, GDK_COLORSPACE_RGB,
      FALSE, 8,  message->width,
      message->height, message->width*3,
      pixbuf_destroyed, NULL);

  received_depth = 1;
  redraw();
}

static void
ipc_kinect_video_handler(carmen_kinect_video_message *message)
{
  int i, j;
  unsigned char *data;

  if (!received_video) {
    gtk_widget_set_usize (drawing_area, message->width*2, message->height);
  } else
    g_object_unref(video);

  data = (unsigned char *)calloc(message->size, sizeof(unsigned char));
  carmen_test_alloc(data);

  for (i = 0; i < message->width*message->height; i++) {
    for (j = 0; j < 3; j++) {
      data[3*i+j] = message->video[3*i+j];
    }
  }

  video = gdk_pixbuf_new_from_data((guchar *)data, GDK_COLORSPACE_RGB,
      FALSE, 8,  message->width,
      message->height, message->width*3,
      pixbuf_destroyed, NULL);

  received_video = 1;
  redraw();
}

static void
shutdown_camera_view(int x)
{
  if(x == SIGINT) {
    carmen_ipc_disconnect();
    printf("Disconnected from robot.\n");
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
expose_event(GtkWidget *widget __attribute__ ((unused)),
    GdkEventExpose *event __attribute__ ((unused)))
{
  redraw();
  return 1;
}



void
generate_snapshot()
{

}


static gint
key_press_event(GtkWidget *widget __attribute__ ((unused)),
    GdkEventKey *key)
{
  if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK)) // TODO: a flag GDK_CONTROL_MASK esta impedindo a captura da tecla
    shutdown_camera_view(SIGINT);

  if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
    shutdown_camera_view(SIGINT);

  if (toupper(key->keyval) == 'S')
    generate_snapshot();

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
redraw(void)
{
  /* Make sure data structures are all the right size. */

  if (received_video){

    gdk_draw_pixbuf(drawing_area->window,
        drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
        video, 0, 0, drawing_area->allocation.width/2, 0,
        drawing_area->allocation.width/2,
        drawing_area->allocation.height,
        GDK_RGB_DITHER_NONE, 0, 0);
  }

  if (received_depth){

    gdk_draw_pixbuf(drawing_area->window,
        drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
        depth, 0, 0, 0, 0,
        drawing_area->allocation.width/2,
        drawing_area->allocation.height,
        GDK_RGB_DITHER_NONE, 0, 0);
  }
}

static void
start_graphics(int argc, char *argv[])
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (main_window), "Kinect Robot View");

  drawing_area = gtk_drawing_area_new ();
  gtk_widget_set_usize (drawing_area, MAX_WIDTH*2, MAX_HEIGHT);

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

int
to_kinect_id(int laser_id)
{
  return laser_id-1;
}

int
to_laser_id(int kinect_id)
{
  return kinect_id+1;
}

int
main(int argc, char **argv)
{
  init();

  carmen_ipc_initialize(argc, argv);

  carmen_kinect_subscribe_depth_message(0, NULL,
      (carmen_handler_t) ipc_kinect_depth_handler,
      CARMEN_SUBSCRIBE_LATEST );

  carmen_kinect_subscribe_video_message(0, NULL,
      (carmen_handler_t) ipc_kinect_video_handler,
      CARMEN_SUBSCRIBE_LATEST );

  signal(SIGINT, shutdown_camera_view);

  start_graphics(argc, argv);

  return 0;
}
