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
#include <carmen/minoru_interface.h>

static GtkWidget *horizontal_container;
static GtkWidget *drawing_area_left;
static GdkPixbuf *image_left;
static GtkWidget *drawing_area_right;
static GdkPixbuf *image_right;

static int received_image = 0;
static void redraw(void);

static int minoru_width;
static int minoru_height;

unsigned char *data_left;
unsigned char *data_right;

carmen_minoru_stereoimage_message message;

static void image_handler(carmen_minoru_stereoimage_message* image_msg)
{
  int i;

  message = *image_msg;

  if (!received_image) {
    gtk_widget_set_usize (drawing_area_left, image_msg->width, image_msg->height);
    gtk_widget_set_usize (drawing_area_right, image_msg->width, image_msg->height);
  } else{
    g_object_unref(image_left);
    g_object_unref(image_right);
  }

  for (i = 0; i < image_msg->image_size; i++)
  {
      data_left[i] = image_msg->raw_left[i];
      data_right[i] = image_msg->raw_right[i];
  }


  image_left = gdk_pixbuf_new_from_data((guchar *)data_left, GDK_COLORSPACE_RGB,
				   FALSE, 8,  image_msg->width,
				   image_msg->height, image_msg->width*3,
				   NULL, NULL);

  image_right = gdk_pixbuf_new_from_data((guchar *)data_right, GDK_COLORSPACE_RGB,
				   FALSE, 8,  image_msg->width,
				   image_msg->height, image_msg->width*3,
				   NULL, NULL);

  received_image = 1;
  redraw();
}

void
save_image (const unsigned char *image, int w, int h, const char *file_name)
{
	FILE *image_file = (FILE *) NULL;
	int i, r, g, b;

	if (!file_name || !image)
		return;

	if ((image_file = fopen (file_name, "w")) == (FILE *) NULL)
		return;

	// Write the image format and comment
	fprintf (image_file, "P3\n# CREATOR: minoru save_image ()\n");

	// Write the image dimentions and range
	fprintf (image_file, "%d %d\n%d\n", w, h, 255);

	for (i = 0; i < h*w; i++)
	{
		r = image[3 * i + 0];
		g = image[3 * i + 1];
		b = image[3 * i + 2];
		fprintf (image_file, "%d\n%d\n%d\n", r, g, b);
	}

	// Closes image file
	fclose (image_file);
}

void compose_output_path(char *dirname, char *filename, char **composed_path)
{
	*composed_path = (char *) malloc (
			(strlen(dirname) + strlen(filename) + 2 /* 1 for '\0' e 1 for the '/' between <dirname>/<filename> */) * sizeof(char));

	sprintf((*composed_path), "%s/%s", dirname, filename);
}

void compose_filename_from_timestamp(double timestamp, char **filename, char *extension)
{
	*filename = (char*) malloc (256 * sizeof(char));
	sprintf((*filename), "%.25f.%s", timestamp, extension);
}

void create_stereo_filename_from_timestamp(double timestamp, char **left_img_filename, char **right_img_filename)
{
	compose_filename_from_timestamp(timestamp, left_img_filename, "l.ppm");
	compose_filename_from_timestamp(timestamp, right_img_filename, "r.ppm");
}

static void
save_camera_images()
{
	char *left_filename, *right_filename;
	char *left_filepath, *right_filepath;
	create_stereo_filename_from_timestamp(message.timestamp, &left_filename, &right_filename);
	compose_output_path("./", left_filename, &left_filepath);
	compose_output_path("./", right_filename, &right_filepath);
	save_image(message.raw_left, message.width, message.height, left_filepath);
	save_image(message.raw_right, message.width, message.height, right_filepath);
}

static void
shutdown_camera_view(int x)
{
	free(data_left);
	free(data_right);

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

  if (toupper(key->keyval) == 'S' && (key->state & GDK_CONTROL_MASK))
    save_camera_images();

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

  gdk_draw_pixbuf(drawing_area_left->window,
		  drawing_area_left->style->fg_gc[GTK_WIDGET_STATE (drawing_area_left)],
		  image_left, 0, 0, 0, 0,
		  drawing_area_left->allocation.width,
		  drawing_area_left->allocation.height,
		  GDK_RGB_DITHER_NONE, 0, 0);

  gdk_draw_pixbuf(drawing_area_right->window,
		  drawing_area_right->style->fg_gc[GTK_WIDGET_STATE (drawing_area_right)],
		  image_right, 0, 0, 0, 0,
		  drawing_area_right->allocation.width,
		  drawing_area_right->allocation.height,
		  GDK_RGB_DITHER_NONE, 0, 0);
}

static void
start_graphics(int argc, char *argv[])
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (main_window), "Minoru Camera");

  horizontal_container = gtk_hbox_new(TRUE, 1);
  drawing_area_left = gtk_drawing_area_new ();
  drawing_area_right = gtk_drawing_area_new ();

  gtk_widget_set_usize (drawing_area_left, minoru_width, minoru_height);
  gtk_widget_set_usize (drawing_area_right, minoru_width, minoru_height);


  gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_left, TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_right, TRUE, TRUE, 0);

  gtk_container_add(GTK_CONTAINER(main_window), horizontal_container);

  gtk_signal_connect(GTK_OBJECT(drawing_area_left), "expose_event",
		     (GtkSignalFunc)expose_event, NULL);
  gtk_signal_connect(GTK_OBJECT(drawing_area_right), "expose_event",
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

  gtk_widget_add_events(drawing_area_right,  GDK_EXPOSURE_MASK
			| GDK_BUTTON_PRESS_MASK
			| GDK_BUTTON_RELEASE_MASK
			| GDK_POINTER_MOTION_MASK
			| GDK_POINTER_MOTION_HINT_MASK
			| GDK_KEY_PRESS_MASK
			| GDK_KEY_RELEASE_MASK);

  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  gtk_widget_show(drawing_area_left);
  gtk_widget_show(drawing_area_right);
  gtk_widget_show(horizontal_container);
  gtk_widget_show(main_window);

  gtk_main();
}

static int
carmen_minoru_read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = {
		{"minoru", "width", CARMEN_PARAM_INT, &minoru_width, 0, NULL},
		{"minoru", "height", CARMEN_PARAM_INT, &minoru_height, 0, NULL},
		};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

int
main(int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);

  carmen_param_check_version(argv[0]);

  carmen_minoru_read_parameters(argc, argv);

  data_left = (unsigned char *)calloc(minoru_width*minoru_height*3, sizeof(unsigned char));
  data_right = (unsigned char *)calloc (minoru_width*minoru_height*3, sizeof(unsigned char));

  signal(SIGINT, shutdown_camera_view);

  carmen_minoru_subscribe_stereoimage(NULL, (carmen_handler_t)image_handler, CARMEN_SUBSCRIBE_LATEST);

  start_graphics(argc, argv);

  return 0;
}
