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
#include <carmen/camera_interface.h>

static GtkWidget *drawing_area;
static GdkPixbuf *image;
static int received_image = 0;
static void redraw(void);

static void pixbuf_destroyed(guchar *pixels, 
			     gpointer data __attribute__ ((unused)))
{
  free(pixels);
}

static void image_handler(carmen_camera_image_message *image_msg)
{
  int i, j;
  unsigned char *data;

  if (!received_image) {
    gtk_widget_set_usize (drawing_area, image_msg->width, image_msg->height);
  } else
    g_object_unref(image);

  data = (unsigned char *)calloc
    (image_msg->width*image_msg->height*3, sizeof(unsigned char));
  carmen_test_alloc(data);

  for (i = 0; i < image_msg->width*image_msg->height; i++) {
    for (j = 0; j < 3; j++) {
      data[3*i+j] = image_msg->image[3*i+j];
    }
  }

  image = gdk_pixbuf_new_from_data((guchar *)data, GDK_COLORSPACE_RGB,
				   FALSE, 8,  image_msg->width, 
				   image_msg->height, image_msg->width*3, 
				   pixbuf_destroyed, NULL);

  received_image = 1;
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

static gint
key_press_event(GtkWidget *widget __attribute__ ((unused)), 
		GdkEventKey *key)
{
  if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK))
    shutdown_camera_view(SIGINT);

  if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
    shutdown_camera_view(SIGINT);
  
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

static void redraw(void)
{
  /* Make sure data structures are all the right size. */

  if (!received_image)
    return;

  gdk_draw_pixbuf(drawing_area->window, 
		  drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
		  image, 0, 0, 0, 0,
		  drawing_area->allocation.width, 
		  drawing_area->allocation.height, 
		  GDK_RGB_DITHER_NONE, 0, 0);
}

static void 
start_graphics(int argc, char *argv[]) 
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (main_window), "Camera View");
  
  drawing_area = gtk_drawing_area_new ();
  gtk_widget_set_usize (drawing_area, 100, 100);

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
main(int argc, char **argv)
{  

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  carmen_camera_subscribe_images(NULL, (carmen_handler_t)image_handler, CARMEN_SUBSCRIBE_LATEST);

  signal(SIGINT, shutdown_camera_view);  

  start_graphics(argc, argv);

  return 0;
}
