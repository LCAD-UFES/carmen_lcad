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

/***********************************
 * map_editor_graphics deals with  *
 * the pixmaps and the main window *
 * which display the map.          *
 ***********************************/

#include <gtk/gtk.h>
#include <carmen/carmen_graphics.h>
#include <carmen/ipc_wrapper.h>
#include <carmen/map_interface.h>

#include "cursors.h"              /* cursor bitmaps */


#include "map_editor.h"
//#include "map_editor_graphics.h"
#include "map_editor_drawing.h"
#include "map_editor_menus.h"

#define DEFAULT_MAX_SCREEN_WIDTH 600
#define DEFAULT_MAX_SCREEN_HEIGHT 400

static GtkObject *x_adjustment, *y_adjustment;
static GtkWidget *drawing_table, *window_box;

static GdkFont *place_font;
static int current_place = -1;
static GtkUIManager *ui_manager;

extern int grayscale;


void create_cursors(void)
{
  static GdkColor color1={0,0xFFFF,0xFFFF,0xFFFF};
  static GdkColor color2={0,0x0000,0x0000,0x0000};
  GdkBitmap *curs_pix, *msk_pix;

  curs_pix = gdk_bitmap_create_from_data(NULL, (const gchar *)point_bits, 
					 point_width, point_height);
  msk_pix = gdk_bitmap_create_from_data(NULL, (const gchar *)pointmsk_bits, 
					pointmsk_width, pointmsk_height);
  cpoint = gdk_cursor_new_from_pixmap(curs_pix, msk_pix, &color2, &color1,
				      3,0);
  gdk_bitmap_unref(curs_pix);
  gdk_bitmap_unref(msk_pix);

  curs_pix = gdk_bitmap_create_from_data(NULL, (const gchar *)pour_bits, 
					 pour_width, pour_height);
  msk_pix = gdk_bitmap_create_from_data(NULL, (const gchar *)pourmsk_bits, 
					pourmsk_width, pourmsk_height);
  cfill = gdk_cursor_new_from_pixmap(curs_pix, msk_pix, &color2, &color1,
				     14, 13);
  gdk_bitmap_unref(curs_pix);
  gdk_bitmap_unref(msk_pix);
  
  curs_pix = gdk_bitmap_create_from_data(NULL, (const gchar *)cross_bits, 
					 cross_width, cross_height);
  msk_pix = gdk_bitmap_create_from_data(NULL, (const gchar *)crossmsk_bits, 
					crossmsk_width, crossmsk_height);
  ccross = gdk_cursor_new_from_pixmap(curs_pix, msk_pix, &color2, &color1,
				      8,8);
  gdk_bitmap_unref(curs_pix);
  gdk_bitmap_unref(msk_pix);
  
  curs_pix = gdk_bitmap_create_from_data(NULL, (const gchar *)select_bits, 
					 select_width, select_height);
  msk_pix = gdk_bitmap_create_from_data(NULL, (const gchar *)selectmsk_bits, 
					selectmsk_width, selectmsk_height);
  cselect = gdk_cursor_new_from_pixmap(curs_pix, msk_pix, &color2, &color1,
				       0,select_height);
  gdk_bitmap_unref(curs_pix);
  gdk_bitmap_unref(msk_pix);
}

static void pixbuf_destroyed(guchar *pixels, 
			     gpointer data __attribute__ ((unused)))
{
  free(pixels);
}

static GdkPixmap *generate_pixmap(unsigned char* image_data,
				  carmen_map_config_p config, 
				  double zoom) 
{
  GdkPixbuf *image, *rotated_image, *final_image;

  if (image_data == NULL) {
    carmen_warn("carmen_graphics_generate_pixmap was passed bad arguments.\n");
    return NULL;
  }

  image = gdk_pixbuf_new_from_data((guchar *)image_data, GDK_COLORSPACE_RGB,
				   FALSE, 8,  config->y_size, config->x_size, 
				   config->y_size*3, pixbuf_destroyed, NULL);

  rotated_image = gdk_pixbuf_rotate_simple(image, GDK_PIXBUF_ROTATE_COUNTERCLOCKWISE);
  g_object_unref(image);  

  final_image = gdk_pixbuf_scale_simple(rotated_image, config->x_size*zoom, 
					config->y_size*zoom, GDK_INTERP_TILES);
  
  g_object_unref(rotated_image);

  if (map_pixmap) 
    gdk_pixmap_unref(map_pixmap);
  map_pixmap = NULL;

  map_pixmap = gdk_pixmap_new(drawing_area->window, config->x_size*zoom, 
			      config->y_size*zoom, -1);
  
  gdk_draw_pixbuf(map_pixmap, 
		  drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
		  final_image, 0, 0, 
		  0, 0, -1, -1, 
		  GDK_RGB_DITHER_NONE, 0, 0);

  g_object_unref(final_image);
  
  return map_pixmap;
}



/*****************************************************************
 * Conversions from the coordinates on the screen to the         *
 * coordinates on the map and vis versa                          *
 *****************************************************************/

/* converts an x position on the drawing area to an x grid
   on the map */
carmen_inline double
pix_x_to_map(double pix_x)
{
  return (double)(pix_x)/mult + (double)xstart;
}

/* converts a y position on the drawing area to a y grid
   on the map */
carmen_inline double 
pix_y_to_map(double pix_y)
{
  return (double)yend - (double)(pix_y)/mult;
}
 
/* converts an x grid to an x position on the drawing area */
carmen_inline double 
map_x_to_pix(int map_x)
{
  return (double)(map_x-xstart)*mult;
}

/* converts a y grid to a y position on the drawing area */
carmen_inline double 
map_y_to_pix(int map_y)
{
  return (double)(yend-map_y)*mult;
}

/* converts an x grid to a position on the map_pixmap */
carmen_inline double 
map_x_to_map_pix(int map_x)
{
  return (double)(map_x)*mult;
}

/* converts an x grid to a position on the map_pixmap */
carmen_inline double 
map_y_to_map_pix(int map_y)
{
  return (double)(map->config.y_size-map_y)*mult;
}


/*****************************************************************
 * Grafics helper functions                                      *
 *****************************************************************/

void 
draw_offlimits(int i)
{
  int pix_x1, pix_y1, pix_x2, pix_y2;

  pix_x1 = map_x_to_pix(offlimits_array[i].x1);
  pix_y1 = map_y_to_pix(offlimits_array[i].y1);
  pix_x2 = map_x_to_pix(offlimits_array[i].x2);
  pix_y2 = map_y_to_pix(offlimits_array[i].y2);
	
  switch (offlimits_array[i].type) {
  case CARMEN_OFFLIMITS_POINT_ID:
    break;
  case CARMEN_OFFLIMITS_LINE_ID:
    gdk_draw_line (tmp_pixmap, drawing_gc, pix_x1, pix_y1, pix_x2, 
		   pix_y2);
    break;
  case CARMEN_OFFLIMITS_RECT_ID:
    gdk_draw_rectangle(tmp_pixmap, drawing_gc, 1, pix_x1, pix_y2, 
		       pix_x2-pix_x1, pix_y1-pix_y2);
    break;
  default:
    break;
  }
}

void 
draw_place_name(int i, GdkPixmap *pixmap, GdkColor *draw_color)
{
  int pix_x1, pix_y1;

  pix_x1 = map_x_to_pix(place_list->places[i].x/map->config.resolution);
  pix_y1 = map_y_to_pix(place_list->places[i].y/map->config.resolution);
	
  gdk_gc_set_foreground (drawing_gc, draw_color);

  gdk_draw_arc(pixmap, drawing_gc, 1, pix_x1-5, pix_y1-5,
	       10, 10, 0, 360*64);

  gdk_gc_set_foreground (drawing_gc, &carmen_black);

  gdk_draw_arc(pixmap, drawing_gc, 0, pix_x1-5, pix_y1-5,
	       10, 10, 0, 360*64); 

  gdk_gc_set_foreground (drawing_gc, &carmen_black);
  gdk_draw_string(pixmap, place_font, drawing_gc, pix_x1, pix_y1-5,
		  place_list->places[i].name);
}

/* draws the map_pixmap onto the tmp_pixmap */
void 
map_to_tmp(void)
{
  
  int xsrc, ysrc, xdest, ydest, w,h;
  xsrc = 0;
  xdest = 0;
  w = xend*mult;
  ysrc = 0;
  ydest = 0;
  h = yend*mult;

  gdk_gc_set_foreground(drawing_gc, &purple);
  gdk_draw_rectangle(tmp_pixmap, drawing_gc, TRUE, 0, 0,
		     drawing_area->allocation.width,
		     drawing_area->allocation.height);

  gdk_draw_pixmap(tmp_pixmap, 
		  drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
		  map_pixmap, xsrc, ysrc, xdest, ydest, w, h);
}


/* sets up the default colors and drawing_gc */
void 
setup_colors(void)
{
  if (drawing_gc == NULL) 
    drawing_gc = gdk_gc_new(drawing_area->window);
  if (drawing_gc == NULL)	
    carmen_die((char *) "drawing_gc could not be initialized\n");

  yellow = carmen_graphics_add_color((char *) "Yellow");
  blue = carmen_graphics_add_color_rgb(0, 0, 255);
  red = carmen_graphics_add_color_rgb(255, 0, 0);
  purple = carmen_graphics_add_color_rgb(150, 0, 150);
}

/* redraws the pixmap onto the drawing area */
void 
redraw(void)
{
  int i;  

  if (drawing_area == NULL || drawing_area->window == NULL)
    return;  

  if(tmp_pixmap == NULL) {
    setup_colors();
    image_data = carmen_graphics_convert_to_image(map, grayscale);
    map_pixmap = generate_pixmap(image_data, &(map->config), mult);

    tmp_pixmap = gdk_pixmap_new(drawing_area->window,
				drawing_area->allocation.width,
				drawing_area->allocation.height,
				-1);
    map_to_tmp();

  }	
  
  if (show_offlimits) {
    gdk_gc_set_foreground (drawing_gc, &red);
    for (i = 0; i < num_offlimits_segments; i++) 
      draw_offlimits(i);
  }
  
  if (show_place_names && place_list) {
    for (i = 0; i < place_list->num_places; i++) 
      draw_place_name(i, tmp_pixmap, &carmen_red);
  }
  
  gdk_draw_pixmap(drawing_area->window,   //dest
		  drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
		  tmp_pixmap,             //src
		  0, 0, 0, 0, -1, -1);

}

/***************************************************************************
 * Event handlers for the drawing area                                     *
 ***************************************************************************/

/* handles a button press event in the drawing area.
   calls the apropriate tool depending upon utensil's setting 
   also creates a backup for undo and incrememnts modified if necessary */
static gint 
button_press_event( GtkWidget *widget, GdkEventButton *event )
{  
  if(tmp_pixmap == NULL)
    return -1;

  if (deleting_placename)
    return TRUE;

  if (adding_placename)
    return TRUE;

  if (adding_door)
    return TRUE;

  if (event->button == 1)
    {
      switch(utensil)
	{
	case CARMEN_MAP_BRUSH:
	  memcpy(backup, map->complete_map, sizeof(float) * map->config.x_size * 
		 map->config.y_size); 
	  modified ++;
	  draw_brush(widget, event->x, event->y);
	  break;
	case CARMEN_MAP_RECTANGLE:
	  memcpy(backup, map->complete_map, sizeof(float) * map->config.x_size * 
		 map->config.y_size);
	  modified ++;
	  creating_rectangle(widget, 0, event->x, event->y);
	  break;
	case CARMEN_MAP_CROP:
	  memcpy(backup, map->complete_map, sizeof(float) * map->config.x_size * 
		 map->config.y_size);
	  modified ++;
	  cropping(widget, 0, event->x, event->y);
	  break;
	case CARMEN_MAP_LINE:
	  memcpy(backup, map->complete_map, sizeof(float) * map->config.x_size * 
		 map->config.y_size);
	  modified ++;
	  creating_line(0, event->x, event->y);
	  break;
	case CARMEN_MAP_FILL:
	  memcpy(backup, map->complete_map, sizeof(float) * map->config.x_size * 
		 map->config.y_size);
	  modified ++;
	  create_fill(event->x, event->y);
	  break;
	case CARMEN_MAP_FUZZY_FILL:
	  memcpy(backup, map->complete_map, sizeof(float) * map->config.x_size * 
		 map->config.y_size);
	  modified ++;
	  create_fuzzy_fill(event->x, event->y);
	  break;
	case CARMEN_MAP_EYEDROPPER:
	  sample(event->x, event->y);
	  break;
	case CARMEN_MAP_ZOOM:
	  zoom_in(event->x, event->y);
	  break;
	case CARMEN_MAP_MOVER:
	  move(0, event->x, event->y);
	default:
	  break;
	}
    }
  else if (event->button == 3 && utensil == CARMEN_MAP_ZOOM)
    zoom_out(event->x, event->y);

  redraw();
  return TRUE;
}

/* handles the mouse button being released inside the drawing area 
   calls the apropriate tool function, or none */
static gint 
button_release_event(GtkWidget *widget, GdkEventButton *event )
{
  double x, y;

  if (deleting_placename) {
    do_delete_placename(current_place);
    return TRUE;
  }
  if (adding_placename) {
    x = pix_x_to_map(event->x)*map->config.resolution;
    y = pix_y_to_map(event->y)*map->config.resolution;
    start_add_placename(x, y);
    return TRUE;
  }
  if (adding_door) {
    x = pix_x_to_map(event->x)*map->config.resolution;
    y = pix_y_to_map(event->y)*map->config.resolution;
    if (adding_door == 2)
      start_add_door(x, y);
    else
      finish_add_door(x, y);
    return TRUE;
  }

  if (event->button == 1 && tmp_pixmap != NULL)
    switch(utensil)
      {
      case CARMEN_MAP_RECTANGLE:
	creating_rectangle(widget, 2, event->x, event->y);
	break;
      case CARMEN_MAP_CROP:
	cropping(widget, 2, event->x, event->y);
	break;
      case CARMEN_MAP_LINE:
	creating_line(2, event->x, event->y);
	break;
      case CARMEN_MAP_MOVER:
	move(2, event->x, event->y);
      default:
	break;
      }

  redraw();
  return 1;
}

static void
handle_deleting_placename_move(int map_x, int map_y)
{
  double closest_distance;
  int closest_place;

  double distance;
  int i;

  closest_distance = hypot(map_x-place_list->places[0].x/
			   map->config.resolution,
			   map_y-place_list->places[0].y/
			   map->config.resolution);
  closest_place = 0;
  for (i = 0; i < place_list->num_places; i++) 
    {
      distance = hypot(map_x-place_list->places[i].x/
		       map->config.resolution,
		       map_y-place_list->places[i].y/
		       map->config.resolution);
      if (distance < closest_distance) 
	{
	  closest_distance = distance;
	  closest_place = i;
	}
    }

  if (closest_place != current_place && current_place >= 0)
    draw_place_name(current_place, tmp_pixmap, &carmen_red);

  current_place = closest_place;
  draw_place_name(closest_place, tmp_pixmap, &carmen_yellow);

}


/* handles the mouse moving in the drawing area
   only calls the appropriate tool function if the mouse button is down.
   always sets the cursor to the appropriate symbol */
static gint 
motion_notify_event( GtkWidget *widget, GdkEventMotion *event )
{
  int x, y;
  GdkModifierType state;
  static char label_buffer[255];
  static carmen_map_point_t map_point;
  static carmen_world_point_t world_point;
	

  gtk_propagate_event(GTK_WIDGET(vrule), (GdkEvent *)event);
  gtk_propagate_event(GTK_WIDGET(hrule), (GdkEvent *)event);

  map_to_tmp();

  if (event->is_hint)
    gdk_window_get_pointer (event->window, &x, &y, &state);
  else
    {
      x = event->x;
      y = event->y;
      state = (GdkModifierType) event->state;
    }
	
  map_point.x = pix_x_to_map(x);
  map_point.y = pix_y_to_map(y);
  map_point.map = map;
  carmen_map_to_world(&map_point, &world_point);
  if ((map_point.x > 0) && (map_point.x < map_point.map->config.x_size) && (map_point.y > 0) && (map_point.y < map_point.map->config.y_size))
    sprintf(label_buffer, "X: %6.2f, Y: %6.2f, V: %lf", world_point.pose.x, world_point.pose.y, world_point.map->map[map_point.x][map_point.y]);
  else
    sprintf(label_buffer, "X: %6.2f, Y: %6.2f", world_point.pose.x, world_point.pose.y);
  gtk_label_set_text(GTK_LABEL(coords_label), label_buffer);

  if (tmp_pixmap == NULL)
    return TRUE;

  if (deleting_placename) 
    {
      handle_deleting_placename_move(map_point.x, map_point.y);
    }
  else if ((state & GDK_BUTTON1_MASK)) 
    {
      switch(utensil)
	{
	case CARMEN_MAP_BRUSH:
	  draw_brush (widget, x, y);
	  x = x/mult - brush_size + .5;
	  y = y/mult - brush_size + .5;
	  break;
	case CARMEN_MAP_RECTANGLE:
	  creating_rectangle(widget, 1, x, y);
	  break;
	case CARMEN_MAP_CROP:
	  cropping(widget, 1, x, y);
	  break;
	case CARMEN_MAP_LINE:
	  creating_line(1, x, y);
	  break;
	case CARMEN_MAP_EYEDROPPER:
	  sample(x, y);
	  break;
	case CARMEN_MAP_MOVER:
	  move(1, x, y);
	default:
	  break;
	}
    }

  redraw();
  
  return TRUE;
}

static gint
configure_event (GtkWidget *widget __attribute__ ((unused)),
		 GdkEventConfigure *event __attribute__ ((unused)))
{
  GtkAdjustment *adjustment;
  /*int width, height, x, y;
  width = event->width;
  height = event->height;
  x = event->x;
  y = event->y;*/

  adjustment = gtk_scrolled_window_get_hadjustment((GtkScrolledWindow *)scrolled_window);
  adjustment->lower = xstart*mult;
  adjustment->upper = xend*mult;

  adjustment->page_increment = (xend-xstart)*mult;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "value_changed");
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");

  //  gtk_ruler_set_range (GTK_RULER (hrule), xstart*mult, xend*mult, xstart*mult, xend*mult);

  adjustment = gtk_scrolled_window_get_vadjustment((GtkScrolledWindow *)scrolled_window);
  adjustment->upper = (map->config.y_size - ystart)*mult;
  adjustment->lower = (map->config.y_size - yend)*mult;

  adjustment->page_increment = (yend-ystart)*mult;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "value_changed");
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");

  if (map_pixmap)
    gdk_pixmap_unref(map_pixmap);
  map_pixmap = NULL;

  if(tmp_pixmap) 
    gdk_pixmap_unref(tmp_pixmap);
  tmp_pixmap = NULL;

  redraw();
  return FALSE;
}


static gint 
expose_event(GtkWidget *widget __attribute__ ((unused)), 
	     GdkEventExpose *event __attribute__ ((unused)))
{
  redraw();
  return FALSE;
}

/*************************************************************************
 * event handlers for the scroll bars -- DEPRICATED                      *
 *************************************************************************/
gint 
horizontal(GtkAdjustment *adj)
     /* depricated */
{
  GtkAdjustment *adjustment;
  int delta;
  int map_x;

  map_x = adj->value/mult;
  delta = (xend-xstart);
  xend = xstart + delta;
  
  gtk_ruler_set_range(GTK_RULER(hrule),
		      map->config.resolution*xstart/100.0, 
		      map->config.resolution*xend/100.0, 
		      map->config.resolution * map_x / 100.0, 
		      map->config.resolution * map->config.x_size / 100.0);
  
  
// This will not happen any more because of scrolled-window technology
  adjustment = gtk_scrolled_window_get_hadjustment((GtkScrolledWindow *)scrolled_window);
  
  if(adjustment->lower/mult < 0 && adjustment->lower/mult < xstart)
    {
      if(xstart > 0)
	adjustment->lower = 0;
      else
	adjustment->lower = xstart*mult;
      gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");
    }
  if(adjustment->upper/mult > map->config.x_size && adjustment->upper/mult > xend)
    {
      if(xend < map->config.x_size)
	adjustment->upper = map->config.x_size*mult;
      else
	adjustment->upper = xend*mult;
      gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");
    }
  

  if(tmp_pixmap)
    {
      map_to_tmp();
      redraw();
    }
  return 1;
}

gint
vertical(GtkAdjustment *adj)
     /* depricated */
{
  GtkAdjustment *adjustment;
  int delta;
  int map_y;

  map_y = map->config.y_size - adj->value/mult;
  delta = (yend-ystart);
  ystart = yend - delta;
  
  gtk_ruler_set_range(GTK_RULER(vrule),
		      map->config.resolution*yend/100.0,  
		      map->config.resolution*ystart/100.0,
		      map->config.resolution * map_y / 100.0, 
		      map->config.resolution * map->config.y_size / 100.0);
  
// This will not happen any more because of scrolled-window technology
    adjustment = gtk_scrolled_window_get_vadjustment((GtkScrolledWindow *)scrolled_window);


  if(map->config.y_size - adjustment->upper/mult < 0 && map->config.y_size - adjustment->upper/mult < ystart)
    {
      if(ystart > 0)// fprintf(stderr,"1");
      	adjustment->upper = map->config.y_size*mult;
      else //fprintf(stderr,"2");
      	adjustment->upper = (map->config.y_size - ystart)*mult;
      gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");
    }
  if(map->config.y_size - adjustment->lower/mult > map->config.y_size && map->config.y_size - adjustment->lower/mult > yend)
    {
      if(yend < map->config.y_size) //fprintf(stderr,"3");
      	adjustment->lower = 0;
      else //fprintf(stderr,"4");
      	adjustment->lower = (map->config.y_size - yend)*mult;
      gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");
    }

    if(tmp_pixmap)
    {
      map_to_tmp();
      redraw();
    }
  
  return 1;
}

/*************************************************************************
 * event handlers for the main window                                    *
 *************************************************************************/

gint 
off_pixmap(void)
{
  if(tmp_pixmap)
    {
      map_to_tmp();
      redraw();
    }
  set_point();
  return 1;
}

/*************************************************************************
 * startup                                                               *
 *************************************************************************/

static GtkActionEntry action_entries[] = {
  {"FileMenu", NULL, "_File", NULL, NULL, NULL},
  {"NewMap", NULL, "_New Map...", "<control>N", NULL, G_CALLBACK(new_map_menu)},
  {"OpenMap", NULL, "_Open Map...", "<control>O", NULL, G_CALLBACK(open_map_menu)},
  {"SaveMap", GTK_STOCK_SAVE, "_Save Map", "<control>S", NULL, G_CALLBACK(save_map_menu)},
  {"SaveMapAs", NULL, "Save Map _As...", NULL, NULL, G_CALLBACK(save_map_as_menu)},
  {"ImportFromBmp", NULL, "Import from BMP", NULL, NULL, G_CALLBACK(import_from_bmp_menu)},
  {"Quit", GTK_STOCK_QUIT, "_Quit", "<control>Q", NULL, G_CALLBACK(quit_menu)},
  {"EditMenu", NULL, "_Edit", NULL, NULL, NULL},
  {"Undo", NULL, "_Undo", "<control>Z", NULL, G_CALLBACK(undo_menu)},
  {"AddPlacename", NULL, "_Add Placename", NULL, NULL, G_CALLBACK(add_placename)},
  {"DeletePlacename", NULL, "_Delete Placename", NULL, NULL, G_CALLBACK(delete_placename)},
  {"AddDoor", NULL, "Add Doo_r", NULL, NULL, G_CALLBACK(add_door)},
  {"ViewMenu", NULL, "_View", NULL, NULL, NULL},
  {"HelpMenu", NULL, "_Help", NULL, NULL, NULL},
  {"About", NULL, "_About", NULL, NULL, G_CALLBACK(help_menu)}};

static GtkToggleActionEntry toggle_entries[] = {
  {"ShowPlacenames", NULL, "Show _Placenames", NULL, NULL, G_CALLBACK(toggle_view), FALSE},
  {"ShowOfflimits", NULL, "Show _Offlimits", NULL, NULL, G_CALLBACK(toggle_view), FALSE}
};

const char *ui_description = 
  "<ui>"
  "  <menubar name='MainMenu'>"
  "    <menu action='FileMenu'>"
  "      <menuitem action='NewMap'/>"
  "      <menuitem action='OpenMap'/>"
  "      <menuitem action='SaveMap'/>"
  "      <menuitem action='SaveMapAs'/>"
  "      <menuitem action='ImportFromBmp'/>"
  "      <separator/>"
  "      <menuitem action='Quit'/>"
  "    </menu>"
  "    <menu action='EditMenu'>"
  "      <menuitem action='Undo'/>"
  "      <separator/>"
  "      <menuitem action='AddPlacename'/>"
  "      <menuitem action='DeletePlacename'/>"
  "      <separator/>"
  "      <menuitem action='AddDoor'/>"
  "    </menu>"
  "    <menu action='ViewMenu'>"
  "      <menuitem action='ShowPlacenames'/>"
  "      <menuitem action='ShowOfflimits'/>"
  "    </menu>"
  "    <menu action='HelpMenu'>"
  "      <menuitem action='About'/>"
  "    </menu>"
  "  </menubar>"
  "</ui>";

static GtkWidget *get_main_menu(void)
{
  GtkWidget *menubar;
  GtkActionGroup *action_group;
  GtkAccelGroup *accel_group;
  GError *error;
  GtkAction* action;

  action_group = gtk_action_group_new ("MenuActions");
  gtk_action_group_add_actions (action_group, action_entries, 
				G_N_ELEMENTS (action_entries), window);
  gtk_action_group_add_toggle_actions (action_group, toggle_entries, 
				       G_N_ELEMENTS (toggle_entries), window);

  ui_manager = gtk_ui_manager_new ();
  gtk_ui_manager_insert_action_group (ui_manager, action_group, 0);
  
  accel_group = gtk_ui_manager_get_accel_group (ui_manager);
  gtk_window_add_accel_group (GTK_WINDOW (window), accel_group);
  
  error = NULL;
  if (!gtk_ui_manager_add_ui_from_string (ui_manager, ui_description, -1, 
					  &error)) {
    g_message ("building menus failed: %s", error->message);
    g_error_free (error);
    exit (EXIT_FAILURE);
  }
  
  menubar = gtk_ui_manager_get_widget (ui_manager, "/MainMenu");

  action = gtk_action_group_get_action(action_group, "ShowPlacenames");
  gtk_toggle_action_set_active(GTK_TOGGLE_ACTION(action), show_place_names);

  action = gtk_action_group_get_action(action_group, "ShowOfflimits");
  gtk_toggle_action_set_active(GTK_TOGGLE_ACTION(action), show_offlimits);

  return menubar;
}

void 
set_up_map_widgets(void)
{ 
  GtkAdjustment *adjustment;

  if (!map)
    return;

  gtk_widget_hide(window);

  xend = map->config.x_size;
  yend = map->config.y_size;

  if(map->config.x_size > map->config.y_size) {
    screen_width = DEFAULT_MAX_SCREEN_WIDTH;
    mult = (double)screen_width / (double)map->config.x_size;
    screen_height = (double)map->config.y_size * (double)mult;
  }
  else {
    screen_height = DEFAULT_MAX_SCREEN_HEIGHT;
    mult = (double)screen_height / (double)map->config.y_size;
    screen_width = (double)map->config.x_size * (double)mult;
    if (screen_width < DEFAULT_MAX_SCREEN_WIDTH)
      screen_width = DEFAULT_MAX_SCREEN_WIDTH;
  }

  gtk_drawing_area_size (GTK_DRAWING_AREA(drawing_area), screen_width, 
			 screen_height);

  gtk_widget_set_usize(drawing_table, screen_width+25, screen_height+25);
  gtk_widget_set_usize(scrolled_window, screen_width+25, screen_height+25);

  /*
  gtk_ruler_set_range(GTK_RULER(hrule),
		      map->config.resolution*xstart/1.0, 
		      map->config.resolution*xend/1.0,  
		      0,
		      map->config.resolution*map->config.x_size / 1.0);
  gtk_ruler_set_range(GTK_RULER(vrule),
		      map->config.resolution*yend/100.0,  
		      map->config.resolution*ystart/100.0,
		      0,
		      map->config.resolution * map->config.y_size / 100.0);
  */
  /*  gtk_ruler_set_range(GTK_RULER(hrule),
      0,drawing_area->allocation.width,0,drawing_area->allocation.width);*/
  
  adjustment = gtk_scrolled_window_get_hadjustment((GtkScrolledWindow *)scrolled_window);
  adjustment->value = 0;
  adjustment->lower = 0;
  adjustment->upper = xend-xstart;
  adjustment->step_increment = 1;
  adjustment->page_increment = xend-xstart;
  adjustment->page_size = xend-xstart;

  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");

  adjustment = gtk_scrolled_window_get_vadjustment((GtkScrolledWindow *)scrolled_window);
  adjustment->value = 0;
  adjustment->lower = 0;
  adjustment->upper = yend-ystart;
  adjustment->step_increment = 1;
  adjustment->page_increment = yend-ystart;
  adjustment->page_size = yend-ystart;

  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");  

  gtk_widget_show_all(window);
}

/* builds the main window and drawing area for the editor */
void start_drawing_window(int *argc, char **argv[])
{
  GtkWidget *box2, *box3, *box4, *box5;
  GtkWidget *separator;
  GtkWidget *text;
  GtkWidget *button;
  GtkWidget *menu_bar;
  GtkObject *adjustment;
  char brush_text[10];
  char ink_text[10];
  char line_text[10];
  char fuzzyness_text[10];

  gtk_init(argc, argv);
  carmen_graphics_setup_colors();
  create_cursors();
  place_font = gdk_font_load ("fixed");

  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

  g_signal_connect(GTK_OBJECT (window), "destroy", G_CALLBACK(gtk_main_quit), (gpointer) "WM destroy");

  gtk_window_set_title (GTK_WINDOW (window), "Map");
  gtk_signal_connect (GTK_OBJECT (window), "delete_event", 
		      GTK_SIGNAL_FUNC (gtk_exit), NULL);
  gtk_container_set_border_width (GTK_CONTAINER (window), 0);

  //  gtk_widget_set_events (window, GDK_POINTER_MOTION_MASK);

  window_box = gtk_vbox_new(FALSE, 0);
  gtk_container_set_border_width (GTK_CONTAINER (window_box), 0);
  gtk_container_add(GTK_CONTAINER(window), window_box);
  gtk_widget_show(window_box);

  /* create menus */
  menu_bar = get_main_menu();
  gtk_box_pack_start(GTK_BOX(window_box), menu_bar, FALSE, TRUE, 0);
  gtk_widget_show(menu_bar);

  /* tools */

  box2 = gtk_hbox_new(FALSE, 2);
  gtk_box_pack_start(GTK_BOX(window_box), box2, FALSE, FALSE, 0);
  gtk_widget_show(box2);
  
  separator = gtk_vseparator_new();
  gtk_box_pack_start(GTK_BOX(box2), separator, FALSE, FALSE, 0);
  gtk_widget_show(separator);

  text = gtk_label_new(" Tools: ");
  gtk_label_set_pattern(GTK_LABEL(text), "_____ ");
  gtk_box_pack_start(GTK_BOX(box2), text, FALSE, FALSE, 0);
  gtk_widget_show(text);

  button = gtk_button_new_with_label (" brush ");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (set_brush), NULL);
  gtk_box_pack_start(GTK_BOX(box2), button, FALSE, FALSE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label (" rectangle ");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (set_rectangle), NULL);
  gtk_box_pack_start(GTK_BOX(box2), button, FALSE, FALSE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label (" line ");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (set_line), NULL);
  gtk_box_pack_start(GTK_BOX(box2), button, FALSE, FALSE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label (" fill ");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (set_fill), NULL);
  gtk_box_pack_start(GTK_BOX(box2), button, FALSE, FALSE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label (" fuzzy fill ");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (set_fuzzy_fill), NULL);
  gtk_box_pack_start(GTK_BOX(box2), button, FALSE, FALSE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label (" eye dropper ");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (set_sample), NULL);
  gtk_box_pack_start(GTK_BOX(box2), button, FALSE, FALSE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label (" zoom ");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (set_zoom), NULL);
  gtk_box_pack_start(GTK_BOX(box2), button, FALSE, FALSE, 0);
  gtk_widget_show(button);
  
  button = gtk_button_new_with_label (" mover ");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (set_mover), NULL);
  gtk_box_pack_start(GTK_BOX(box2), button, FALSE, FALSE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label (" crop ");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (set_crop), NULL);
  gtk_box_pack_start(GTK_BOX(box2), button, FALSE, FALSE, 0);
  gtk_widget_show(button);

  cur_label_label = gtk_label_new(" current tool: ");
  gtk_box_pack_start(GTK_BOX(box2), cur_label_label, TRUE, TRUE, 0);
  gtk_widget_show(cur_label_label);
  
  tool_label = gtk_label_new("brush");
  gtk_box_pack_start(GTK_BOX(box2), tool_label, TRUE, TRUE, 0);
  gtk_widget_show(tool_label);
  set_brush();

  coords_label = gtk_label_new("X:    0 Y:    0 ");
  gtk_box_pack_start(GTK_BOX(box2), coords_label, FALSE, FALSE, 0);
  gtk_widget_show(coords_label);

  separator = gtk_hseparator_new();
  gtk_widget_set_usize(separator, 700, 15);
  gtk_box_pack_start(GTK_BOX(window_box), separator, FALSE, FALSE, 0);
  gtk_widget_show(separator);

  /* table */
  drawing_table = gtk_table_new(2,2,FALSE);
  gtk_widget_set_usize(drawing_table, 225, 225);
  gtk_box_pack_start(GTK_BOX(window_box), drawing_table, TRUE, TRUE, 0);

  /* (m) */
  text = gtk_label_new("m");
  gtk_table_attach(GTK_TABLE(drawing_table), text,
		   0,1,0,1,
		   GTK_SHRINK,
		   GTK_SHRINK,
                   1,1);
 
  /* Create Scrolled-Window */
  scrolled_window = gtk_scrolled_window_new (NULL,NULL);
  gtk_scrolled_window_set_policy (GTK_SCROLLED_WINDOW (scrolled_window),
                                  GTK_POLICY_ALWAYS, GTK_POLICY_ALWAYS);
  gtk_table_attach(GTK_TABLE(drawing_table), scrolled_window,
		   1,2,1,2,
		   (GtkAttachOptions) (GTK_EXPAND|GTK_SHRINK|GTK_FILL),
		   (GtkAttachOptions) (GTK_EXPAND|GTK_SHRINK|GTK_FILL),
                   1,1);
  gtk_widget_set_usize(scrolled_window, 225, 225);
  gtk_widget_show (scrolled_window);

  /* Create the drawing area */

  drawing_area = gtk_drawing_area_new ();
  gtk_drawing_area_size (GTK_DRAWING_AREA (drawing_area),200,200);

  gtk_widget_set_events (drawing_area, GDK_EXPOSURE_MASK
			 | GDK_BUTTON_PRESS_MASK
			 | GDK_POINTER_MOTION_MASK
			 | GDK_BUTTON_RELEASE_MASK);

  gtk_scrolled_window_add_with_viewport (
                   GTK_SCROLLED_WINDOW (scrolled_window), drawing_area);
  gtk_widget_show (drawing_area);

  /* scale */
  hrule = gtk_hruler_new();
  vrule = gtk_vruler_new();
  gtk_ruler_set_metric( GTK_RULER(hrule), GTK_PIXELS);
  gtk_ruler_set_metric( GTK_RULER(vrule), GTK_PIXELS);

  gtk_table_attach(GTK_TABLE(drawing_table), hrule,
		   1,2,0,1,
		   (GtkAttachOptions) (GTK_FILL|GTK_EXPAND|GTK_SHRINK),
		   GTK_FILL,
                   1,1);
  gtk_table_attach(GTK_TABLE(drawing_table), vrule,
		   0,1,1,2,
		   GTK_FILL,
		   (GtkAttachOptions) (GTK_EXPAND|GTK_SHRINK|GTK_FILL),
                   1,1);

  gtk_widget_show (hrule);
  gtk_widget_show (vrule);

  /* create the scroll bars for the drawing area */

  x_adjustment = (GTK_OBJECT (gtk_scrolled_window_get_hadjustment((GtkScrolledWindow *)scrolled_window)));

  //  gtk_signal_connect (GTK_OBJECT (x_adjustment), "value_changed",
  //		      GTK_SIGNAL_FUNC (horizontal), NULL);
  //  gtk_signal_connect (GTK_OBJECT (x_adjustment), "changed",
  //		      GTK_SIGNAL_FUNC (horizontal), NULL);
  
  y_adjustment = (GTK_OBJECT (gtk_scrolled_window_get_vadjustment((GtkScrolledWindow *)scrolled_window)));

  //  gtk_signal_connect (GTK_OBJECT (y_adjustment), "value_changed",
  //		      GTK_SIGNAL_FUNC (vertical), NULL);
  // gtk_signal_connect (GTK_OBJECT (y_adjustment), "changed",
  //		      GTK_SIGNAL_FUNC (vertical), NULL);
  
  /* Signals used to handle backing pixmap */
  gtk_signal_connect (GTK_OBJECT (drawing_area), "expose_event", 
		      (GtkSignalFunc) expose_event, NULL);
  gtk_signal_connect (GTK_OBJECT (drawing_area), "button_press_event",
		      (GtkSignalFunc) button_press_event, NULL);
  gtk_signal_connect (GTK_OBJECT (drawing_area), "motion_notify_event",
		      (GtkSignalFunc) motion_notify_event, NULL);
  gtk_signal_connect (GTK_OBJECT (drawing_area), "button_release_event",
		      (GtkSignalFunc) button_release_event, NULL);
  gtk_signal_connect (GTK_OBJECT(drawing_area),"configure_event",
		      (GtkSignalFunc) configure_event, NULL); 

  separator = gtk_hseparator_new();
  gtk_box_pack_start(GTK_BOX(window_box), separator, FALSE, FALSE, 0);
  gtk_widget_show(separator);

  /* buttons */
  box2 = gtk_hbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(window_box), box2, FALSE, FALSE, 0);
  gtk_widget_show(box2);

  separator = gtk_vseparator_new();
  gtk_box_pack_start(GTK_BOX(box2), separator, FALSE, FALSE, 5);
  gtk_widget_show(separator);

  /*ink */
  box3 = gtk_vbox_new(TRUE, 0);
  gtk_box_pack_start(GTK_BOX(box2), box3, TRUE, TRUE, 0);
  gtk_widget_show(box3);

  box4 = gtk_hbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(box3), box4, TRUE, TRUE, 0);
  gtk_widget_show(box4);
	
  box5 = gtk_vbox_new(TRUE, 0);
  gtk_box_pack_start(GTK_BOX(box4), box5, TRUE, TRUE, 0);
  gtk_widget_show(box5);

  button = gtk_button_new_with_label ("unknown");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", 
		      GTK_SIGNAL_FUNC (set_unknown), NULL);
  gtk_box_pack_start(GTK_BOX(box5), button, FALSE, TRUE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label ("offlimits");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", 
		      GTK_SIGNAL_FUNC (set_offlimits), NULL);
  gtk_box_pack_start(GTK_BOX(box5), button, FALSE, TRUE, 0);
  gtk_widget_show(button);

  if(ink < -1) {
    ink = .90;
    color = carmen_graphics_add_color_rgb(255.0*(1.0-ink),
					  255.0*(1.0-ink), 
					  255.0*(1.0-ink));
  }

  adjustment = gtk_adjustment_new(ink, 0.0, 1.0, 0.01, 0.1, 0.0);
  gtk_signal_connect (GTK_OBJECT (adjustment), "value_changed",
		      GTK_SIGNAL_FUNC (set_ink), NULL);
  gtk_signal_connect (GTK_OBJECT (adjustment), "changed",
		      GTK_SIGNAL_FUNC (set_ink), NULL);
  ink_scale = gtk_hscale_new(GTK_ADJUSTMENT(adjustment));
  gtk_scale_set_draw_value(GTK_SCALE(ink_scale), FALSE);

  box5 = gtk_vbox_new(TRUE, 0);
  gtk_box_pack_start(GTK_BOX(box4), box5, TRUE, TRUE, 0);
  gtk_widget_show(box5);
  
  text = gtk_label_new("Ink (Probability)");
  gtk_box_pack_start(GTK_BOX(box5), text, TRUE, TRUE, 0);
  gtk_widget_show(text);
  
  gtk_box_pack_start(GTK_BOX(box5), ink_scale, TRUE, TRUE, 0);
  gtk_widget_show(ink_scale);
  
  sprintf(ink_text, "%.2f", ink);
  ink_label = gtk_label_new(ink_text);
  gtk_box_pack_start(GTK_BOX(box5), ink_label, TRUE, TRUE, 0);
  gtk_widget_show(ink_label);

  separator = gtk_vseparator_new();
  gtk_box_pack_start(GTK_BOX(box2), separator, FALSE, FALSE, 5);
  gtk_widget_show(separator);

  /* fuzzyness */
  box3 = gtk_vbox_new(TRUE, 0);
  gtk_box_pack_start(GTK_BOX(box2), box3, TRUE, TRUE, 0);
  gtk_widget_show(box3);

  text = gtk_label_new("Fuzziness");
  gtk_box_pack_start(GTK_BOX(box3), text, TRUE, TRUE, 0);
  gtk_widget_show(text);

  adjustment = gtk_adjustment_new(fuzzyness, 0.0, 1.0, 0.01, 0.1, 0.0);
  gtk_signal_connect (GTK_OBJECT (adjustment), "value_changed",
		      GTK_SIGNAL_FUNC (set_fuzzyness), NULL);
  fuzzy_scale = gtk_hscale_new(GTK_ADJUSTMENT(adjustment));
  gtk_scale_set_draw_value(GTK_SCALE(fuzzy_scale), FALSE);

  gtk_box_pack_start(GTK_BOX(box3), fuzzy_scale, TRUE, TRUE, 0);
  gtk_widget_show(fuzzy_scale);

  sprintf(fuzzyness_text, "%.2f", fuzzyness);
  fuzzyness_label = gtk_label_new(fuzzyness_text);
  gtk_box_pack_start(GTK_BOX(box3), fuzzyness_label, TRUE, TRUE, 0);
  gtk_widget_show(fuzzyness_label);

  separator = gtk_vseparator_new();
  gtk_box_pack_start(GTK_BOX(box2), separator, FALSE, FALSE, 5);
  gtk_widget_show(separator);

  /* brush size */
  box3 = gtk_vbox_new(TRUE, 0);
  gtk_box_pack_start(GTK_BOX(box2), box3, TRUE, TRUE, 0);
  gtk_widget_show(box3);

  text = gtk_label_new("Brush size");
  gtk_box_pack_start(GTK_BOX(box3), text, TRUE, TRUE, 0);
  gtk_widget_show(text);

  box4 = gtk_hbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(box3), box4, TRUE, TRUE, 0);
  gtk_widget_show(box4);

  /* brush decr */
  button = gtk_button_new_with_label ("smaller");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (brush_decr), NULL);
  gtk_box_pack_start(GTK_BOX(box4), button, TRUE, TRUE, 0);
  gtk_widget_show(button);

  /* brush incr */
  button = gtk_button_new_with_label ("larger");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (brush_incr), NULL);
  gtk_box_pack_start(GTK_BOX(box4), button, TRUE, TRUE, 0);
  gtk_widget_show(button);

  sprintf(brush_text, "%d", carmen_trunc(2.0 * brush_size));
  brush_label = gtk_label_new(brush_text);
  gtk_box_pack_start(GTK_BOX(box3), brush_label, FALSE, FALSE, 0);
  gtk_widget_show(brush_label);

  separator = gtk_vseparator_new();
  gtk_box_pack_start(GTK_BOX(box2), separator, FALSE, FALSE, 5);
  gtk_widget_show(separator);

  /* line_size */
  box3 = gtk_vbox_new(TRUE, 0);
  gtk_box_pack_start(GTK_BOX(box2), box3, TRUE, TRUE, 0);
  gtk_widget_show(box3);

  text = gtk_label_new("Line size");
  gtk_box_pack_start(GTK_BOX(box3), text, TRUE, TRUE, 0);
  gtk_widget_show(text);

  box4 = gtk_hbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(box3), box4, TRUE, TRUE, 0);
  gtk_widget_show(box4);

  button = gtk_button_new_with_label ("smaller");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (line_decr), NULL);
  gtk_box_pack_start(GTK_BOX(box4), button, TRUE, TRUE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label ("larger");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (line_incr), NULL);
  gtk_box_pack_start(GTK_BOX(box4), button, TRUE, TRUE, 0);
  gtk_widget_show(button);

  sprintf(line_text, "%d", carmen_trunc(line_size));
  line_label = gtk_label_new(line_text);
  gtk_box_pack_start(GTK_BOX(box3), line_label, FALSE, FALSE, 0);
  gtk_widget_show(line_label);
  
  separator = gtk_vseparator_new();
  gtk_box_pack_start(GTK_BOX(box2), separator, FALSE, FALSE, 5);
  gtk_widget_show(separator);

  /* filled */
  box3 = gtk_vbox_new(TRUE, 0);
  gtk_box_pack_start(GTK_BOX(box2), box3, TRUE, TRUE, 0);
  gtk_widget_show(box3);

  text = gtk_label_new("Shape fill");
  gtk_box_pack_start(GTK_BOX(box3), text, TRUE, TRUE, 0);
  gtk_widget_show(text);

  box4 = gtk_hbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(box3), box4, TRUE, TRUE, 0);
  gtk_widget_show(box4);

  button = gtk_button_new_with_label ("not filled");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", 
		      GTK_SIGNAL_FUNC (set_not_filled), NULL);
  gtk_box_pack_start(GTK_BOX(box4), button, TRUE, TRUE, 0);
  gtk_widget_show(button);

  button = gtk_button_new_with_label ("filled");
  gtk_signal_connect (GTK_OBJECT (button), "clicked", 
		      GTK_SIGNAL_FUNC (set_filled), NULL);
  gtk_box_pack_start(GTK_BOX(box4), button, TRUE, TRUE, 0);
  gtk_widget_show(button);

  fill_label = gtk_label_new("not filled");
  gtk_box_pack_start(GTK_BOX(box3), fill_label, FALSE, FALSE, 0);
  gtk_widget_show(fill_label);

  separator = gtk_vseparator_new();
  gtk_box_pack_start(GTK_BOX(box2), separator, FALSE, FALSE, 5);
  gtk_widget_show(separator);

  if (map)
    set_up_map_widgets();
  else
    gtk_widget_show(window);

}

