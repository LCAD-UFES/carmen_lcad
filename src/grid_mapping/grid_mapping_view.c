#include <carmen/carmen_graphics.h>
#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/grid_mapping_interface.h>
#include <prob_measurement_model.h>
#include <prob_map.h>

// Image show
static GtkWidget *drawing_area;
static GdkPixbuf *map_buffer;

// Init parameter to reconstruct the map
static int log_odds_bias = 100;

static void
shutdown_grid_mapping_view(int x)
{
  if(x == SIGINT) {
    carmen_verbose("Disconnecting Grid Mapping View Service.\n");
    exit(1);
  }
}


static void
pixbuf_destroyed(guchar *pixels,
    gpointer data __attribute__ ((unused)))
{
  free(pixels);
}


static void redraw_viewer(void)
{
  gdk_draw_pixbuf(drawing_area->window,
                  drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
                  map_buffer, 0, 0, 0, 0,
                  drawing_area->allocation.width,
                  drawing_area->allocation.height,
                  GDK_RGB_DITHER_NONE, 0, 0);
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
  	shutdown_grid_mapping_view(SIGINT);

  if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
  	shutdown_grid_mapping_view(SIGINT);

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
  carmen_ipc_sleep(0.01);
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  return 1;
}

void
save_carmen_map(const carmen_map_t carmen_map, int index, int log_odds_bias)
{
	int x, y;
	char map_file_name[1000];
	FILE *map_file;

	sprintf(map_file_name, "map_file%d.pnm", index);
	map_file = fopen(map_file_name, "w");

	// PNM file header
	fprintf(map_file, "P3\n#PNM criado por Alberto\n");
	fprintf(map_file, "%d %d\n255\n", carmen_map.config.x_size, carmen_map.config.y_size);

	for (y = carmen_map.config.y_size - 1; y >= 0; y--)
	{
		for (x = 0; x < carmen_map.config.x_size; x++)
		{
			double map_value = (double)carmen_map.complete_map[x * carmen_map.config.y_size + y];
			int gray_value;
			if (map_value == 1.0)
			{
				gray_value = PMC_OBSTACLE_COLOR_LIMIT;
			}
			else if (map_value == 0.0)
			{
				gray_value = PMC_WHITE;
			}
			else if (map_value < 0)
			{
				gray_value = PMC_UNKNOWN_AREA;
			}
			else
			{
				double yy = (255.0 * (1.0 - map_value));
				double log_odds = (double)(log(yy / (255.0 - yy)) + (double)log_odds_bias);
				double p_mi = 1.0 - (1.0 / (1.0 + exp((double)(log_odds - (double)log_odds_bias))));
				gray_value = (int)(p_mi * 255.0);
			}

			fprintf(map_file, "%d\n%d\n%d\n", gray_value, gray_value, gray_value);
		}
	}

	fclose(map_file);
}

static void
grid_mapping_handler(carmen_grid_mapping_message *message)
{
	static char *image_data;
	int x, y, i;

	if (message->size <= 0)
		return;

	gtk_widget_set_usize(drawing_area, message->config.x_size, message->config.y_size);

	if (!image_data)
		image_data = (char*)malloc(3 * message->size * sizeof(char));

	// Copy the received map data to a carmen_map
	static carmen_map_t carmen_map;
	carmen_map.complete_map = message->complete_map;
	carmen_map.config = message->config;
	if (!carmen_map.map)
		carmen_map.map=(double **)calloc(message->config.x_size, sizeof(double *));
	copy_complete_map_to_map(&carmen_map, message->config);

	// copy the map to a three channel image buffer
	i = 0;
	for (y = message->config.y_size - 1; y >= 0; y--)
	{
		for (x = 0; x < message->config.x_size; x++)
		{
			double map_value = message->complete_map[x * message->config.y_size + y];
			int gray_value;

			if (map_value == 1.0)
			{
				gray_value = PMC_OBSTACLE_COLOR_LIMIT;
			}
			else if (map_value == 0.0)
			{
				gray_value = PMC_WHITE;
			}
			else if (map_value < 0.0)
			{
				gray_value = PMC_UNKNOWN_AREA;
			}
			else
			{
				double yy = (255.0 * (1.0 - map_value));
				double log_odds = (double)(log(yy / (255.0 - yy)) + (double)log_odds_bias);
				double p_mi = 1.0 - (1.0 / (1.0 + exp((double)(log_odds - (double)log_odds_bias))));
				gray_value = (int)(p_mi * 255.0);
			}

			image_data[3 * i + 0] = gray_value;
			image_data[3 * i + 1] = gray_value;
			image_data[3 * i + 2] = gray_value;
			i++;
		}
	}

	// copy the image buffer to a GDK buffer
	map_buffer = gdk_pixbuf_new_from_data((guchar*)image_data, GDK_COLORSPACE_RGB,
	                                                FALSE, 8, message->config.x_size,
	                                                message->config.y_size, 3 * message->config.x_size,
	                                                pixbuf_destroyed, NULL);

	redraw_viewer();
}

static void
start_graphics(int argc, char *argv[])
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (main_window), "Grid Mapping View");

  drawing_area = gtk_drawing_area_new ();

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

/* read all parameters from .ini file and command line. */
void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
			{"grid_mapping", "map_log_odds_bias", CARMEN_PARAM_INT, &log_odds_bias, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

int
main(int argc, char **argv)
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_grid_mapping_view);

  /* Initialize all the relevant parameters */
  read_parameters(argc, argv);

  /* Subscribe to Grid Mapping Service */
  carmen_grid_mapping_subscribe_message(NULL,
  		(carmen_handler_t)grid_mapping_handler,
			CARMEN_SUBSCRIBE_LATEST);

  /* Start the graphics windows */
  start_graphics(argc, argv);

  return 0;
}
