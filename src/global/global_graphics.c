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

#include "carmen.h"
#include <gtk/gtk.h>
#include "global.h"
#include <carmen/map.h>
#include "global_graphics.h"
#include <sys/types.h>

#ifdef __APPLE__
#include <limits.h>
#include <float.h>
#define MAXDOUBLE DBL_MAX
#else
#include <values.h>
#endif 

fd_set *x_ipcGetConnections(void);
int x_ipcGetMaxConnection(void);

GdkColor carmen_red, carmen_blue, carmen_white, carmen_yellow, carmen_green, 
  carmen_light_blue, carmen_black, carmen_orange, carmen_grey, 
  carmen_light_grey, carmen_purple, carmen_light_green;

GdkColor carmen_colors[12];


void carmen_graphics_update_ipc_callbacks(GdkInputFunction callback_Func) 
{
  fd_set *open_fds;
  int max_connection;
  int index;
  int callback_index;
  carmen_graphics_callback *callback;
  carmen_graphics_callback new_callback;
  static carmen_list_t *existing_callbacks = NULL;  
  static int num_callbacks = 0;

  if (existing_callbacks == NULL)
    existing_callbacks = 
      carmen_list_create(sizeof(carmen_graphics_callback), 10);

  for (index = 0; index < num_callbacks; index++) {
    callback = carmen_list_get(existing_callbacks, index);
    callback->ok = 0;
  }

  open_fds = x_ipcGetConnections();
  max_connection = x_ipcGetMaxConnection();
  for (index = 0; index <= max_connection; index++) {
    if (FD_ISSET(index, open_fds)) {
      for (callback_index = 0; callback_index < num_callbacks; 
	   callback_index++) {
	callback = carmen_list_get(existing_callbacks, callback_index);
	if (index == callback->fd) {
	  callback->ok = 1;
	  break;
	}
      } 
      if (callback_index == existing_callbacks->length) {
	new_callback.fd = index;
	new_callback.ok = 1;
	new_callback.callback_id = 
	  gdk_input_add(index, GDK_INPUT_READ, callback_Func, NULL);
	carmen_list_add(existing_callbacks, &new_callback);
      }
    } /* End of if (FD_ISSET(index, open_fds)) */
  } /* End of for (index = 0; index <= max_connection; index++) */

  for (index = 0; index < num_callbacks; index++) {
    callback = carmen_list_get(existing_callbacks, index);
    if (callback->ok == 0) {
      gdk_input_remove(callback->callback_id);
      carmen_list_delete(existing_callbacks, index);
      index--;
    }
  }
}

static GdkColormap *cmap = NULL;

static void _add_color(GdkColor *color, char *name)
{
  if(cmap == NULL)
    cmap = gdk_colormap_get_system();

  if (!gdk_color_parse (name, color)) {
    g_error("couldn't parse color");
    return;
  }

  if(!gdk_colormap_alloc_color(cmap, color, FALSE, TRUE))
    g_error("couldn't allocate color");
}

void
carmen_graphics_setup_colors(void)
{
  _add_color(&carmen_red, "red");
  _add_color(&carmen_blue, "blue");
  _add_color(&carmen_white, "white");
  _add_color(&carmen_yellow, "yellow");
  _add_color(&carmen_green, "green");
  _add_color(&carmen_light_blue, "DodgerBlue");
  _add_color(&carmen_black, "black");
  _add_color(&carmen_orange, "tomato");
  _add_color(&carmen_grey, "ivory4");
  _add_color(&carmen_light_grey, "grey79");
  _add_color(&carmen_purple, "purple");
  _add_color(&carmen_light_green, "light green");

  carmen_colors[CARMEN_RED] = carmen_red;
  carmen_colors[CARMEN_BLUE] = carmen_blue;
  carmen_colors[CARMEN_WHITE] = carmen_white;
  carmen_colors[CARMEN_YELLOW] = carmen_yellow;
  carmen_colors[CARMEN_GREEN] = carmen_green;
  carmen_colors[CARMEN_LIGHT_BLUE] = carmen_light_blue;
  carmen_colors[CARMEN_BLACK] = carmen_black;
  carmen_colors[CARMEN_ORANGE] = carmen_orange;
  carmen_colors[CARMEN_GREY] = carmen_light_grey;
  carmen_colors[CARMEN_LIGHT_GREY] = carmen_red;
  carmen_colors[CARMEN_PURPLE] = carmen_purple;
  carmen_colors[CARMEN_LIGHT_GREEN] = carmen_light_green;
}

GdkColor carmen_graphics_add_color(char *name) 
{
  GdkColor color;

  _add_color(&color, name);
  return color;
}

GdkColor carmen_graphics_add_color_rgb(int r, int g, int b) 
{
  GdkColor color;

  if(cmap == NULL)
    cmap = gdk_colormap_get_system();

  color.red = r * 256;
  color.green = g * 256;
  color.blue = b * 256;

  if(!gdk_colormap_alloc_color(cmap, &color, FALSE, TRUE))
    g_error("couldn't allocate color");

  return color;
}

#define ELLIPSE_POINTS 30

void carmen_graphics_draw_ellipse(GdkPixmap *pixmap, GdkGC *GC, double x, 
				  double y, double a, double b, double c, 
				  double k);

/* There was a copy of draw_ellipse from localizegraph.c here, but it takes
   things in funny units. x/y should be world points, and a/b/c should also
   be defined with respect to the world, not the map. This needs to be
   fixed. */

static void write_pixbuf_as_png(GdkPixbuf *pixbuf, char *user_filename) 
{
  char basefilename[100] = "video";
  char filename[1024];
  GError *error;
  static int image_count = 0;
  
  if (user_filename == NULL)
    sprintf(filename, "%s%04d.png", basefilename, image_count);
  else
    snprintf(filename, 1023, "%s", user_filename);

  carmen_verbose("Saving image to %s... ", filename);
  
  error = NULL;
  gdk_pixbuf_save(pixbuf, filename, "png", &error, NULL);

  image_count++;
}

void carmen_graphics_write_pixmap_as_png(GdkPixmap *pixmap, 
					 char *user_filename, 
					 int x, int y, int w, int h)
{
  GdkPixbuf *pixbuf;
  pixbuf = gdk_pixbuf_get_from_drawable(NULL, pixmap, NULL,
					x, y, 0, 0, w, h);
  write_pixbuf_as_png(pixbuf, user_filename);
  g_object_unref(pixbuf);
}

/* generates a pixmap from Image_Data */
static void pixbuf_destroyed(guchar *pixels, 
			     gpointer data __attribute__ ((unused)))
{
  free(pixels);
}

void carmen_graphics_write_data_as_png(unsigned char *data, 
				       char *user_filename, int w, int h)
{
  GdkPixbuf *image, *rotated_image;

  image = gdk_pixbuf_new_from_data((guchar *)data, GDK_COLORSPACE_RGB,
				    FALSE, 8, h, w, h*3, pixbuf_destroyed, 
				    NULL);

  rotated_image = gdk_pixbuf_rotate_simple
    (image, GDK_PIXBUF_ROTATE_COUNTERCLOCKWISE);
  g_object_unref(image);  

  write_pixbuf_as_png(rotated_image, user_filename);
  g_object_unref(rotated_image);
}


/* image to map */

int carmen_map_image_to_map_color_unknown(unsigned char r, unsigned char g, 
					  unsigned char b) 
{
  if (   ( r == 0 && g == 0 &&   b==255) ||
	 ( r == 0 && g == 255 && b==0) )
    return 1;
  return 0;
}

float carmen_map_image_to_map_color_occupancy(unsigned char r, unsigned char g,
					      unsigned char b) 
{
  float occ =( ((float)r) + ((float)g) + ((float)b)  ) / (3.0 * 255.0);
  if (occ < 0.001)
    occ = 0.001;
  if (occ > 0.999)
    occ = 0.999;
  return 1.0 - occ;
}

#ifndef COMPILE_WITHOUT_MAP_SUPPORT
unsigned char* carmen_graphics_convert_to_image(carmen_map_p map, int flags)
{
//	static double last_timestamp = 0.0;
//	double time = carmen_get_time();

	double *data_ptr;
	unsigned char *image_data = NULL;
	unsigned char *image_ptr = NULL;
	double value;
	int x_size, y_size;
	int x_index, y_index;
	int index;
	double max_val = -MAXDOUBLE, min_val = MAXDOUBLE;
	unsigned char blue, green, red;
	road_prob *cell;

	int rescale = flags & CARMEN_GRAPHICS_RESCALE;
	int invert = flags & CARMEN_GRAPHICS_INVERT;
	int rotate = flags & CARMEN_GRAPHICS_ROTATE;
	int black_and_white = flags & CARMEN_GRAPHICS_BLACK_AND_WHITE;
	int enhance_contrast = flags & CARMEN_GRAPHICS_ENHANCE_CONTRAST;
	int grayscale = flags & CARMEN_GRAPHICS_GRAYSCALE;
	int remove_minus_one = flags & CARMEN_GRAPHICS_REMOVE_MINUS_ONE;
	int log_odds = flags & CARMEN_GRAPHICS_LOG_ODDS;
	int road_contrast = flags & CARMEN_GRAPHICS_ROAD_CONTRAST;
	int enhace_low_prob_contrast = flags & CARMEN_GRAPHICS_ENHACE_LOW_PROB_CONTRAST;

	if (map == NULL)
	{
		carmen_warn("carmen_graphics_convert_to_image was passed NULL map.\n");
		return NULL;
	}

	x_size = map->config.x_size;
	y_size = map->config.y_size;
	image_data = (unsigned char*) calloc(x_size * y_size * 3, sizeof(unsigned char));
	carmen_test_alloc(image_data);

	if (rescale || grayscale)
	{
		max_val = -MAXDOUBLE;
		min_val = MAXDOUBLE;
		data_ptr = map->complete_map;
		for (index = 0; index < map->config.x_size * map->config.y_size; index++)
		{
			max_val = carmen_fmax(max_val, *data_ptr);
			min_val = carmen_fmin(min_val, *data_ptr);
			data_ptr++;
		}
	}

	if (max_val < 0)
		rescale = 0;

	image_ptr = image_data;
	data_ptr = map->complete_map;
	for (x_index = 0; x_index < x_size; x_index++)
	{
		for (y_index = 0; y_index < y_size; y_index++)
		{
			value = *(data_ptr++);
			if (rotate)
				image_ptr = image_data + y_index * x_size * 3 + x_index;
			if (grayscale)
			{
				value = (value - min_val) / (max_val - min_val);
				for (index = 0; index < 3; index++)
					*(image_ptr++) = value * 255;
			}
			else if (log_odds)
			{
				double p = 1.0L - (1.0L / (1.0L + expl((long double) value)));
				if (invert)
					p = 1.0 - p;
				for (index = 0; index < 3; index++)
					*(image_ptr++) = p * 255;
			}
			else if (road_contrast)
			{
				cell = road_mapper_double_to_prob(&value);
				road_mapper_cell_color(cell, &blue, &green, &red);
				*(image_ptr++) = red;
				*(image_ptr++) = green;
				*(image_ptr++) = blue;
			}
			else if (value < 0 && value > -1.5)
			{
				if (black_and_white)
				{
					*(image_ptr++) = 255;
					*(image_ptr++) = 255;
					*(image_ptr++) = 255;
				}
				else
				{
					*(image_ptr++) = 30;
					*(image_ptr++) = 144;
					*(image_ptr++) = 255;
				}
			}
			else if (value < -1.5)
			{ // for offlimits
				if (black_and_white)
				{
					*(image_ptr++) = 205;
					*(image_ptr++) = 205;
					*(image_ptr++) = 205;
				}
				else
				{
					*(image_ptr++) = 255;
					*(image_ptr++) = 0;
					*(image_ptr++) = 0;
				}
			}
			else if (!rescale && value > 1.0)
			{
				if (black_and_white)
				{
					*(image_ptr++) = 128;
					*(image_ptr++) = 128;
					*(image_ptr++) = 128;
				}
				else
				{
					*(image_ptr++) = 255;
					*(image_ptr++) = 0;
					*(image_ptr++) = 0;
				}
			}
			else
			{
				if (remove_minus_one)
				{
					min_val = 0.0;
					if (value == -1.0)
						value = 0.0;
				}
				if (rescale)
					value = (value - min_val) / (max_val - min_val);
				if (!invert)
					value = 1 - value;
				if (enhance_contrast)
					value = carmen_clamp(0.0, value * 3.0, 1.0);
				if (enhace_low_prob_contrast)
				{
					if ((value != -1.0) && (value < 0.3))
						value = 0.0;
				}
				for (index = 0; index < 3; index++)
					*(image_ptr++) = value * 255;
			}
		}
	}

//	printf("time - last_timestamp = %lf,  dt %lf\n", time - last_timestamp, time - carmen_get_time());
//	last_timestamp = carmen_get_time();

	return image_data;
}

GdkPixmap * 
carmen_graphics_generate_pixmap(GtkWidget* drawing_area, 
				unsigned char* data,
				carmen_map_config_p config, 
				double zoom) 
{
  GdkPixbuf *image, *rotated_image, *final_image;
  GdkPixmap *pixmap;
  int x_render_size, y_render_size;

  if (drawing_area == NULL || data == NULL) {
    carmen_warn("carmen_graphics_generate_pixmap was passed bad arguments.\n");
    return NULL;
  }

  image = gdk_pixbuf_new_from_data((guchar *)data, GDK_COLORSPACE_RGB,
				    FALSE, 8, config->y_size, config->x_size, 
				    config->y_size*3, pixbuf_destroyed, NULL);
  rotated_image = gdk_pixbuf_rotate_simple
    (image, GDK_PIXBUF_ROTATE_COUNTERCLOCKWISE);
  g_object_unref(image);

  x_render_size = zoom*config->x_size;
  y_render_size = zoom*config->y_size;

  final_image = gdk_pixbuf_scale_simple(rotated_image, x_render_size, 
					y_render_size, GDK_INTERP_BILINEAR);
  pixmap = gdk_pixmap_new
      (drawing_area->window, x_render_size, y_render_size, -1);

  gdk_draw_pixbuf(pixmap, drawing_area->style->fg_gc
		  [GTK_WIDGET_STATE (drawing_area)],
		  final_image, 0, 0, 0, 0, -1, -1, 
		  GDK_RGB_DITHER_NONE, 0, 0);

  g_object_unref(rotated_image);  
  g_object_unref(final_image);  

  return pixmap;
}

carmen_map_p carmen_pixbuf_to_map(GdkPixbuf* pixbuf, double resolution )
{
  carmen_map_p map;
  int x_index, y_index;
  int rowstride, n_channels;
  guchar *pixels, *p;

  if (pixbuf == NULL) {
    carmen_warn("Error: im = NULL in %s at line %d in file %s\n",
		__FUNCTION__, __LINE__, __FILE__);
    return NULL;
  }

  if (gdk_pixbuf_get_colorspace (pixbuf) != GDK_COLORSPACE_RGB) {
    carmen_warn("File is not RGB colorspace. carmen_pixbuf_to_map failed\n");
    return NULL;
  }

  if (gdk_pixbuf_get_bits_per_sample (pixbuf) != 8) {
    carmen_warn("File is not 8 bits per pixel. carmen_pixbuf_to_map failed\n");
    return NULL;
  }

  if (gdk_pixbuf_get_has_alpha (pixbuf) == 1) {
    carmen_warn("File has alpha channel. carmen_pixbuf_to_map failed\n");
    return NULL;
  }

  n_channels = gdk_pixbuf_get_n_channels (pixbuf);
  if (n_channels != 3) {
    carmen_warn("File has alpha channel. carmen_pixbuf_to_map failed\n");
    return NULL;
  }

  map = (carmen_map_p)calloc(1, sizeof(carmen_map_t));
  carmen_test_alloc(map);

  map->config.x_size = gdk_pixbuf_get_width(pixbuf);
  map->config.y_size = gdk_pixbuf_get_height(pixbuf);
  map->config.resolution =  resolution;

  map->complete_map = (double *)
    calloc(map->config.x_size*map->config.y_size, sizeof(double));
  carmen_test_alloc(map->complete_map);
  
  map->map = (double **)calloc(map->config.x_size, sizeof(double *));
  carmen_test_alloc(map->map);
  for (x_index = 0; x_index < map->config.x_size; x_index++) 
    map->map[x_index] = map->complete_map+x_index*map->config.y_size;

  rowstride = gdk_pixbuf_get_rowstride (pixbuf);
  pixels = gdk_pixbuf_get_pixels (pixbuf);

  for (x_index = 0; x_index < map->config.x_size; x_index++) 
    for (y_index = 0; y_index < map->config.y_size; y_index++) {
      unsigned char r,g ,b;
      p = pixels + y_index * rowstride + x_index * n_channels;
      r = p[0];
      g = p[1];
      b = p[2];
      
      if (carmen_map_image_to_map_color_unknown(r,g,b)) 
	map->map[x_index][map->config.y_size-1-y_index] = -1.0;
      else 
	map->map[x_index][map->config.y_size-1-y_index] =  
	  carmen_map_image_to_map_color_occupancy(r,g,b);
    }
  return map;
}

carmen_map_p carmen_map_imagefile_to_map(char *filename, double resolution) 
{
  carmen_map_p map;
  GdkPixbuf* buf;
  GError *error;

  error = NULL;
  buf = gdk_pixbuf_new_from_file(filename, &error);
  if (buf == NULL)
    carmen_die_syserror("Couldn't open %s for reading", filename);

  map = carmen_pixbuf_to_map(buf, resolution);

  g_object_unref(buf);

 return map;
}
#endif
