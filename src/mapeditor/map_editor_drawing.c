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

/*************************************************
 * map_editor_drawing implements all the drawing *
 * based features of the editor                  *
 *************************************************/

#include <gtk/gtk.h>
#include <carmen/carmen.h>

GdkColor carmen_graphics_add_color_rgb(int r, int g, int b);

#include "map_editor.h"
#include "map_editor_drawing.h"
#include "map_editor_graphics.h"
#include "map_editor_menus.h"

typedef struct temp_node carmen_stack_t;
struct temp_node{
  int x,y;
  carmen_stack_t *next;
};

/* calculates the angle from horizontal (in radians) of a vector from (x1, y1) to (x2, y2) */
double 
calc_theta(double x_1, double y_1, double x_2, double y_2)
{
  double dx = x_2 - x_1;
  double dy = y_2 - y_1;
  double hyp = hypot(dx, dy);
  double theta;
  if(hyp == 0)
    return 0;
  theta = asin(dy/hyp);
  if(dx < 0)
    theta = M_PI - theta;
  return theta;
}

/*****************************************************
 * drawing settings                                  *
 *****************************************************/

/* sets the cursor to a pointer */
gint 
set_point(void)
{
  if (window->window != NULL)
    gdk_window_set_cursor (window->window, cpoint);
  return 1;
}

/* sets the cursor to a cross */
gint 
set_cross(void)
{
  if (window->window != NULL)
    gdk_window_set_cursor (window->window, ccross);
  return 1;
}

/* sets the pointer to a paint can */
gint 
set_pour(void)
{
  if (window->window != NULL)
    gdk_window_set_cursor (window->window, cfill);
  return 1;
}

/* sets the pointer to an eye dropper (sort of) */
gint 
set_dropper(void)
{
  if (window->window != NULL)
    gdk_window_set_cursor (window->window, cselect);
  return 1;
}

/* sets the ink based upon an adjustment to the ink slider */
gint 
set_ink(GtkAdjustment *adj)
{
  char ink_text[10];
  ink = adj->value;
  color = carmen_graphics_add_color_rgb(255.0*(1.0-ink), 
					255.0*(1.0-ink),
					255.0*(1.0-ink));
  sprintf(ink_text, "%.2f", ink);
  gtk_label_set_text(GTK_LABEL(ink_label), ink_text);
  return 1;
}

/* sets the ink to unknown */
gint 
set_unknown(void)
{
  ink = -1;
  color = blue;
  gtk_label_set_text(GTK_LABEL(ink_label), "unknown");
  return 1;
}

gint 
set_offlimits(void)
{
  ink = -2;
  color = red;
  gtk_label_set_text(GTK_LABEL(ink_label), "offlimits");
  drawing_offlimits = 1;
  return 1;
}

/* sets the fuzzyness for fuzzy fill */
gint 
set_fuzzyness(GtkAdjustment *adj)
{
  char fuzzyness_text[10];

  fuzzyness = adj->value;
  sprintf(fuzzyness_text, "%.2f", fuzzyness);
  gtk_label_set_text(GTK_LABEL(fuzzyness_label), fuzzyness_text);
  return 1;
}

/* sets the tool to a brush */
gint 
set_brush(void)
{
  set_point();
  utensil = CARMEN_MAP_BRUSH;
  gtk_label_set_text(GTK_LABEL(tool_label), "brush");
  return 1;
}

/* sets the tool to something that makes a rectangle */
gint 
set_rectangle(void)
{
  set_cross();
  utensil = CARMEN_MAP_RECTANGLE;
  gtk_label_set_text(GTK_LABEL(tool_label), "rectangle");
  return 1;
}

gint 
set_crop(void)
{
  set_cross();
  utensil = CARMEN_MAP_CROP;
  gtk_label_set_text(GTK_LABEL(tool_label), "crop");
  return 1;
}

/* sets the tool to a line drawer */
gint 
set_line(void)
{
  set_cross();
  utensil = CARMEN_MAP_LINE;
  gtk_label_set_text(GTK_LABEL(tool_label), "line");
  return 1;
}

/* sets the tool to a paint can */
gint 
set_fill(void)
{
  set_pour();
  utensil = CARMEN_MAP_FILL;
  gtk_label_set_text(GTK_LABEL(tool_label), "fill");
  return 1;
}

/* sets the tool to a paint can */
gint 
set_fuzzy_fill(void)
{
  set_pour();
  utensil = CARMEN_MAP_FUZZY_FILL;
  gtk_label_set_text(GTK_LABEL(tool_label), "fuzzy fill");
  return 1;
}

/* sets the tool to a eye dropper */
gint 
set_sample(void)
{
  set_dropper();
  utensil = CARMEN_MAP_EYEDROPPER;
  gtk_label_set_text(GTK_LABEL(tool_label), "eye dropper");
  return 1;
}

/* sets the tool to a eye dropper */
gint 
set_zoom(void)
{
  set_point();
  utensil = CARMEN_MAP_ZOOM;
  gtk_label_set_text(GTK_LABEL(tool_label), "zoom");
  return 1;
}

/* sets the tool to a mover */
gint
set_mover(void)
{
  utensil = CARMEN_MAP_MOVER;
  gtk_label_set_text(GTK_LABEL(tool_label), "mover");
  return 1;
}

/* sets shapes (rectangle) to be filled */
gint 
set_filled(void)
{
  filled = 1;
  gtk_label_set_text(GTK_LABEL(fill_label), "filled");
  return 1;
}

/* sets shapes (rectangle) to not be filled */
gint 
set_not_filled(void)
{
  filled = 0;
  gtk_label_set_text(GTK_LABEL(fill_label), "not filled");
  return 1;
}

/* increments the line size */
gint 
line_incr(void)
{
  char line_text[10];
  line_size ++;
  
  sprintf(line_text, "%d", carmen_trunc(line_size));
  gtk_label_set_text(GTK_LABEL(line_label), line_text);
  return 1;
}

/* decrements the line size */
gint 
line_decr(void)
{
  char line_text[10];
  line_size --;
  if(line_size < 1)
    line_size = 1;
  
  sprintf(line_text, "%d", carmen_trunc(line_size));
  gtk_label_set_text(GTK_LABEL(line_label), line_text);
  return 1;
}

/* decrememnts the size of the brush */
gint 
brush_decr(void)
{
  char brush_text[10];
  brush_size --;
  if(brush_size < .5)
    brush_size = .5;
  
  sprintf(brush_text, "%d", carmen_trunc(2.0 * brush_size));
  gtk_label_set_text(GTK_LABEL(brush_label), brush_text);
  return 1;
}

/* incrememnts the size of the brush */
gint 
brush_incr(void)
{
  char brush_text[10];
  brush_size ++;
  
  sprintf(brush_text, "%d", carmen_trunc(2.0 * brush_size));
  gtk_label_set_text(GTK_LABEL(brush_label), brush_text);
  return 1;
}

static void
check_offlimits_array_size(void)
{
  if (num_offlimits_segments == offlimits_capacity)
    {
      if (offlimits_capacity == 0) 
	{
	  offlimits_capacity = 20;
	  offlimits_array = (carmen_offlimits_p)calloc
	    (offlimits_capacity, sizeof(carmen_offlimits_t));
	  carmen_test_alloc(offlimits_array);
	}
      else
	{
	  offlimits_capacity *= 2;
	  offlimits_array = (carmen_offlimits_p)realloc
	    (offlimits_array, offlimits_capacity*sizeof(carmen_offlimits_t));
	  carmen_test_alloc(offlimits_array);
	}
    }
}

/**********************************************************************
 * functions to do the actual drawing on the pixmap and float * map   *
 *********************************************************************/

/* draws a square of size 2 * brush_size on a side onto the pixmap and 
   the map */
void 
draw_brush( GtkWidget *widget __attribute__ ((unused)), 
	    gdouble pix_x, gdouble pix_y)
{
  int x_left, y_top, x, y;

  GdkRectangle update_rect;

  gdk_gc_set_foreground (drawing_gc, &color);

  x_left = pix_x_to_map(pix_x) - brush_size + .5;
  y_top  = pix_y_to_map(pix_y) + brush_size + .5;
  update_rect.x = map_x_to_map_pix(x_left) + 1.0;
  update_rect.y = map_y_to_map_pix(y_top ) + 1.0;
  update_rect.width  = map_x_to_map_pix((double)(x_left) + brush_size * 2.0) -
    update_rect.x + 1.0;
  if(update_rect.width < 1)
    update_rect.width = 1;
  update_rect.height = map_y_to_map_pix((double)(y_top ) - brush_size * 2.0) -
    update_rect.y + 1.0;
  if(update_rect.height < 1)
    update_rect.height = 1;
  gdk_draw_rectangle (map_pixmap,
		      drawing_gc,
		      TRUE,
		      update_rect.x, update_rect.y,
		      update_rect.width, update_rect.height);

  map_to_tmp();
	
  for(x = x_left; x < x_left + 2.0 * brush_size; x ++)
    if(x < map->config.x_size && x >= 0)
      for(y = y_top - 2.0 * brush_size; y < y_top ; y++)
	if(y < map->config.y_size && y >= 0)
	  map->map[x][y] = ink;
}

/* draws a rectangle (either filled or not depending upon whether filled is 
   set or not) with line widths of line_size onto the pixmap and the map */
void 
draw_rectangle(GtkWidget *widget __attribute__ ((unused)), 
	       gdouble pix_x1, gdouble pix_y1, gdouble pix_x2, gdouble pix_y2)
{
  int i, j;
  double width, height;
  int map_x1, map_x2, map_y1, map_y2;

  if(pix_x1 < pix_x2)
    {
      map_x1 = pix_x_to_map(pix_x1) + .60;
      map_x2 = pix_x_to_map(pix_x2) + .40;
    }
  else
    {
      map_x1 = pix_x_to_map(pix_x2) + .60;
      map_x2 = pix_x_to_map(pix_x1) + .40;
    }
  if(pix_y1 > pix_y2)
    {
      map_y1 = pix_y_to_map(pix_y1) + .60;
      map_y2 = pix_y_to_map(pix_y2) + .40;
    }
  else
    {
      map_y1 = pix_y_to_map(pix_y2) + .60;
      map_y2 = pix_y_to_map(pix_y1) + .40;
    }
  
  gdk_gc_set_foreground (drawing_gc, &color);

  if (drawing_offlimits)
    {
      check_offlimits_array_size();
      offlimits_array[num_offlimits_segments].type = CARMEN_OFFLIMITS_RECT_ID;
      offlimits_array[num_offlimits_segments].x1 = map_x1;
      offlimits_array[num_offlimits_segments].y1 = map_y1;
      offlimits_array[num_offlimits_segments].x2 = map_x2;
      offlimits_array[num_offlimits_segments].y2 = map_y2;

      num_offlimits_segments++;
			
      gdk_gc_set_foreground (drawing_gc, &red);
      draw_offlimits(num_offlimits_segments-1);

      map_to_tmp();
    }
  
  else if(filled)
    {
      for(i = map_x1; i < map_x2; i++)
	if(i < map->config.x_size && i >= 0)
	  for(j = map_y1; j < map_y2; j++)
	    if(j < map->config.y_size && j >= 0)
	      map->map[i][j] = ink;
      width  = map_x_to_map_pix(map_x2) - (int)(map_x_to_map_pix(map_x1));
      if(width < 1)
	width = 1;
      height = map_y_to_map_pix(map_y1) - (int)(map_y_to_map_pix(map_y2));
      if(height < 1)
	height = 1;

      gdk_draw_rectangle (map_pixmap, drawing_gc, 1, 
			  map_x_to_map_pix(map_x1) + 1.0, 
			  map_y_to_map_pix(map_y2) + 1.0, 
			  width, height);
      map_to_tmp();
    }
  else
    {
      //left line
      width = map_x_to_map_pix(map_x1) - 
	(int)(map_x_to_map_pix(map_x1-line_size));
      height =map_y_to_map_pix(map_y1) - (int)(map_y_to_map_pix(map_y2));
      if(height < 1)
	height = 1;
      
      if(width > map_x_to_map_pix(map_x2) - map_x_to_map_pix(map_x1))
	{
	  for(j = map_y1; j < map_y2; j++)
	    if(j>= 0 && j <map->config.y_size)
	      for(i = map_x1; i < map_x2; i++)
		if(i >= 0 && i < map->config.x_size)
		  map->map[i][j] = ink;
	  
	  width = map_x_to_map_pix(map_x2) - (int)(map_x_to_map_pix(map_x1));
	  if(width < 1)
	    width = 1;
	  gdk_draw_rectangle (map_pixmap, drawing_gc, 1,
			      map_x_to_map_pix(map_x1) + 1.0, 
			      map_y_to_map_pix(map_y2) + 1.0, 
			      width, height);
	}
      else
	{
	  for(j = map_y1; j < map_y2; j++)
	    if(j>= 0 && j <map->config.y_size)
	      for(i = map_x1; i < map_x1 + line_size; i++)
		if(i >= 0 && i < map->config.x_size)
		  map->map[i][j] = ink;
	  
	  if(width < 1)
	    width = 1;
	  gdk_draw_rectangle (map_pixmap, drawing_gc, 1,
			      map_x_to_map_pix(map_x1) + 1.0, 
			      map_y_to_map_pix(map_y2) + 1.0, 
			      width, height);
	  //right line
	  width = map_x_to_map_pix(map_x2) - 
	    (int)(map_x_to_map_pix(map_x2-line_size));
	  if(width < 1)
	    width = 1;
	  
	  for(j = map_y1; j < map_y2; j++)
	    if(j>= 0 && j <map->config.y_size)
	      for(i = map_x2 - line_size; i < map_x2; i++)
		if(i >= 0 && i < map->config.x_size)
		  map->map[i][j] = ink;
	  gdk_draw_rectangle (map_pixmap, drawing_gc, 1,
			      map_x_to_map_pix(map_x2-line_size) + 1.0, 
			      map_y_to_map_pix(map_y2) + 1.0, width, height);
	  
	}
      //bottom line
      width = map_x_to_map_pix(map_x2) - (int)(map_x_to_map_pix(map_x1));
      if(width < 1)
	width = 1;
      height =map_y_to_map_pix(map_y1) - (int)(map_y_to_map_pix(map_y1+line_size));

      if(height > map_y_to_map_pix(map_y1) - map_y_to_map_pix(map_y2))
	{
	  for(i = map_x1; i < map_x2; i++)
	    if(i >= 0 && i <map->config.x_size)
	      for(j = map_y1; j < map_y2; j++)
		if(j >= 0 && j < map->config.y_size)
		  map->map[i][j] = ink;

	  height = map_y_to_map_pix(map_y1) - (int)(map_y_to_map_pix(map_y2));
	  if(height < 1)
	    height = 1;
	  gdk_draw_rectangle (map_pixmap, drawing_gc, 1,
			      map_x_to_map_pix(map_x1) + 1.0, map_y_to_map_pix(map_y2) + 1.0, 
			      width, map_y_to_map_pix(map_y1) - map_y_to_map_pix(map_y2));
	}
      else
	{

	  for(i = map_x1; i < map_x2; i++)
	    if(i >= 0 && i <map->config.x_size)
	      for(j = map_y1; j < map_y1 + line_size; j++)
		if(j >= 0 && j < map->config.y_size)
		  map->map[i][j] = ink;
	  
	  if(height < 1)
	    height = 1;
	  gdk_draw_rectangle (map_pixmap, drawing_gc, 1,
			      map_x_to_map_pix(map_x1) + 1.0, 
			      map_y_to_map_pix(map_y1+line_size) + 1.0, width, height);
	  //top line
	  height =map_y_to_map_pix(map_y2) - (int)(map_y_to_map_pix(map_y2+line_size));
	  if(height < 1)
	    height = 1;
	  
	  for(i = map_x1; i < map_x2; i++)
	    if(i >= 0 && i <map->config.x_size)
	      for(j = map_y2 - line_size; j < map_y2; j++)
		if(j >= 0 && j < map->config.y_size)
		  map->map[i][j] = ink;
	  gdk_draw_rectangle (map_pixmap, drawing_gc, 1,
			      map_x_to_map_pix(map_x1) + 1.0, map_y_to_map_pix(map_y2) + 1.0, 
			      width, height);
	}
      map_to_tmp();
    }
}

/* remembers the starting corner of the rectangle and draws a temporary 
   rectangle onto the drawing area while the rectangle is being drawn, and 
   then calls draw_rectangle to draw the final rectangle */
void 
creating_rectangle(GtkWidget *widget, int state, double pix_x, double pix_y)
{
  int x_1, x_2, y_1, y_2;
  static double xs, ys;
  switch(state)
    {
    case 0:
      xs = pix_x;
      ys = pix_y;
      break;
    case 1:
      if(xs < 0)
	{
	  xs = pix_x;
	  ys = pix_y;
	  return;
	}
      if(xs < pix_x)
	{
	  x_1 = pix_x_to_map(xs) + .60;
	  x_2 = pix_x_to_map(pix_x) + .40;
	}
      else
	{
	  x_1 = pix_x_to_map(pix_x) + .60;
	  x_2 = pix_x_to_map(xs) + .40;
	}
      if(ys > pix_y)
	{
	  y_1 = pix_y_to_map(ys) + .60;
	  y_2 = pix_y_to_map(pix_y) + .40;
	}
      else
	{
	  y_1 = pix_y_to_map(pix_y) + .60;
	  y_2 = pix_y_to_map(ys) + .40;
	}
      
      x_1 = map_x_to_pix(x_1);
      x_2 = map_x_to_pix(x_2);
      y_1 = map_y_to_pix(y_1);
      y_2 = map_y_to_pix(y_2);
      if(x_1 >= x_2)
	x_2 = x_1 + 2;
      if(y_2 >= y_1)
	y_1 = y_2 +2;
      map_to_tmp();
      if (x_1 < x_2 && y_2 < y_1)
	{
	  gdk_gc_set_foreground (drawing_gc, &yellow);
	  gdk_draw_rectangle (tmp_pixmap, drawing_gc, 0,
			      x_1 + 1.0, y_2 + 1.0, 
			      x_2 - x_1 - 1.0, 
			      y_1 - y_2 - 1.0);
	}
      //redraw();
      break;
    case 2:
      draw_rectangle(widget, xs, ys, pix_x, pix_y);
      xs = -1;
      ys = -1; 
      break;
    }
}

void crop(double pix_x1, double pix_y1, double pix_x2, double pix_y2)
{
  int i;
  int map_x1, map_x2, map_y1, map_y2;
  double dx, dy;

  if(pix_x1 < pix_x2) {
    map_x1 = pix_x_to_map(pix_x1) + .60;
    map_x2 = pix_x_to_map(pix_x2) + .40;
  } else {
    map_x1 = pix_x_to_map(pix_x2) + .60;
    map_x2 = pix_x_to_map(pix_x1) + .40;
  }

  if(pix_y1 > pix_y2) {
    map_y1 = pix_y_to_map(pix_y1) + .60;
    map_y2 = pix_y_to_map(pix_y2) + .40;
  } else {
    map_y1 = pix_y_to_map(pix_y2) + .60;
    map_y2 = pix_y_to_map(pix_y1) + .40;
  }
  
  for (i = 0; i < map_x2 - map_x1; i++) {
    memmove(&map->complete_map[i*(map_y2-map_y1)],
	    &map->complete_map[(map_x1+i)*map->config.y_size + map_y1],
	    (map_y2 - map_y1)*sizeof(float));
    map->map[i] = &map->complete_map[i*(map_y2-map_y1)];
  }
  
  map->config.x_size = map_x2 - map_x1;
  map->config.y_size = map_y2 - map_y1;

  dx = map_x1 * map->config.resolution;
  dy = map_y1 * map->config.resolution;

  for (i = 0; place_list && i < place_list->num_places; i++) {
    if (place_list->places[i].x < map_x1 * map->config.resolution ||
	place_list->places[i].x >= map_x2 * map->config.resolution ||
	place_list->places[i].y < map_y1 * map->config.resolution ||
	place_list->places[i].y >= map_y2 * map->config.resolution)
      do_delete_placename(i--);
    else {
      place_list->places[i].x -= dx;
      place_list->places[i].y -= dy;
    }
  }

  for (i = 0; i < num_offlimits_segments; i++) {
    offlimits_array[i].x1 -= map_x1;
    offlimits_array[i].x2 -= map_x1;
    offlimits_array[i].y1 -= map_y1;
    offlimits_array[i].y2 -= map_y1;
  }

  if(map_pixmap) {
    gdk_pixmap_unref(map_pixmap);
    gdk_pixmap_unref(tmp_pixmap);
  }

  map_pixmap = NULL;
  tmp_pixmap = NULL;

  set_up_map_widgets();
}

void 
cropping(GtkWidget *widget __attribute__ ((unused)), int state, double pix_x, double pix_y)
{
  int x_1, x_2, y_1, y_2;
  static double xs, ys;
  switch(state)
    {
    case 0:
      xs = pix_x;
      ys = pix_y;
      break;
    case 1:
      if(xs < 0)
	{
	  xs = pix_x;
	  ys = pix_y;
	  return;
	}
      if(xs < pix_x)
	{
	  x_1 = pix_x_to_map(xs) + .60;
	  x_2 = pix_x_to_map(pix_x) + .40;
	}
      else
	{
	  x_1 = pix_x_to_map(pix_x) + .60;
	  x_2 = pix_x_to_map(xs) + .40;
	}
      if(ys > pix_y)
	{
	  y_1 = pix_y_to_map(ys) + .60;
	  y_2 = pix_y_to_map(pix_y) + .40;
	}
      else
	{
	  y_1 = pix_y_to_map(pix_y) + .60;
	  y_2 = pix_y_to_map(ys) + .40;
	}
      
      x_1 = map_x_to_pix(x_1);
      x_2 = map_x_to_pix(x_2);
      y_1 = map_y_to_pix(y_1);
      y_2 = map_y_to_pix(y_2);
      if(x_1 >= x_2)
	x_2 = x_1 + 2;
      if(y_2 >= y_1)
	y_1 = y_2 +2;
      map_to_tmp();
      if (x_1 < x_2 && y_2 < y_1)
	{
	  gdk_gc_set_foreground (drawing_gc, &yellow);
	  gdk_draw_rectangle (tmp_pixmap, drawing_gc, 0,
			      x_1 + 1.0, y_2 + 1.0, 
			      x_2 - x_1 - 1.0, 
			      y_1 - y_2 - 1.0);
	}
      //redraw();
      break;
    case 2:
      crop(xs, ys, pix_x, pix_y);
      xs = -1;
      ys = -1; 
      break;
    }
}

/* draws a line of width line_size onto the map and the pixmap */
void 
draw_line(double pix_x1, double pix_y1, double pix_x2, double pix_y2)
{
  carmen_bresenham_param_t width, length;
  int map_x1, map_x2, map_y1, map_y2;
  int x, y, w, h;
  double theta;

  theta = calc_theta(pix_x1, pix_y1, pix_x2, pix_y2);

  map_x1 = pix_x_to_map(pix_x1-line_size/2.0*cos(theta + M_PI)*mult) + .5;
  map_x2 = pix_x_to_map(pix_x2-line_size/2.0*cos(theta + M_PI)*mult) + .5;
  map_y1 = pix_y_to_map(pix_y1-line_size/2.0*sin(theta + M_PI)*mult) + .5;
  map_y2 = pix_y_to_map(pix_y2-line_size/2.0*sin(theta + M_PI)*mult) + .5;

  if (drawing_offlimits)
    {
      check_offlimits_array_size();
      offlimits_array[num_offlimits_segments].type = CARMEN_OFFLIMITS_LINE_ID;
      offlimits_array[num_offlimits_segments].x1 = map_x1;
      offlimits_array[num_offlimits_segments].y1 = map_y1;
      offlimits_array[num_offlimits_segments].x2 = map_x2;
      offlimits_array[num_offlimits_segments].y2 = map_y2;

      num_offlimits_segments++;
      gdk_gc_set_foreground (drawing_gc, &red);
      draw_offlimits(num_offlimits_segments-1);

      map_to_tmp();
      return;
    }
  
  if(map_x1 == map_x2 && map_y1 == map_y2)
    {
      if(map_x1 >= 0 && map_x1 < map->config.x_size && map_y1 >= 0 && 
	 map_y1 < map->config.y_size)
	{
	  map->map[map_x1][map_y1] = ink;
	  w = map_x_to_map_pix(map_x1+line_size) - 
	    (int)(map_x_to_map_pix(map_x1));
	  if(w < 1)
	    w = 1;
	  h = map_y_to_map_pix(map_y1+line_size) - 
	    (int)(map_y_to_map_pix(map_y1));
	  if(h < 1)
	    h = 1;
	  gdk_draw_rectangle(map_pixmap, drawing_gc, 1,
			     map_x_to_map_pix(map_x1-1.0) + 1.0, 
			     map_y_to_map_pix(map_y1+1.0) + 1.0, 
			     w, h);
	}
    }
  else
    {
      theta = calc_theta(map_x1, map_y1, map_x2, map_y2);
      carmen_get_bresenham_parameters(map_x1, map_y1, map_x2, map_y2, &length);
      
      gdk_gc_set_foreground (drawing_gc, &color);
      do
	{
	  carmen_get_current_point(&length, &x, &y);
	  
	  map_x1 = x;
	  map_x2 = line_size * cos(theta + M_PI/ 2.0) + x + .5;
	  map_y1 = y;
	  map_y2 = line_size * sin(theta + M_PI/ 2.0) + y + .5;
	  if(map_x1 == map_x2 && map_y1 == map_y2)
	    {
	      if(x >= 0 && x < map->config.x_size && y >= 0 && 
		 y < map->config.y_size)
		{
		  w = map_x_to_map_pix(x+1) - (int)(map_x_to_map_pix(x));
		  if(w < 1)
		    w = 1;
		  h = map_y_to_map_pix(y) - (int)(map_y_to_map_pix(y+1));
		  if(h < 1)
		    h = 1;
		  map->map[x][y] = ink;
		  gdk_draw_rectangle(map_pixmap, drawing_gc, 1,
				     map_x_to_map_pix(x) + 1.0, 
				     map_y_to_map_pix(y+1) + 1.0, w, h);
		}
	    }
	  
	  else
	    {
	      carmen_get_bresenham_parameters(map_x1, map_y1, map_x2, map_y2, 
					      &width);
	      do
		{
		  carmen_get_current_point(&width, &x, &y);
		  
		  if(x >= 0 && x < map->config.x_size && y >= 0 && 
		     y < map->config.y_size)
		    {
		      w = map_x_to_map_pix(x+1) - (int)(map_x_to_map_pix(x));
		      if(w < 1)
			w = 1;
		      h = map_y_to_map_pix(y) - (int)(map_y_to_map_pix(y+1));
		      if(h < 1)
			h = 1;
		      map->map[x][y] = ink;
		      gdk_draw_rectangle(map_pixmap, drawing_gc, 1,
					 map_x_to_map_pix(x) + 1.0, 
					 map_y_to_map_pix(y+1) + 1.0, w,h);
		    }
		  
		}
	      while(carmen_get_next_point(&width));
	      
	      carmen_get_bresenham_parameters(map_x1+1.0 * cos(theta+M_PI/ 4.0), 
					      map_y1+1.0 * sin(theta+M_PI/ 4.0), 
					      map_x2+1.0 * cos(theta+M_PI/ 4.0), 
					      map_y2+1.0 * sin(theta+M_PI/ 4.0), 
					      &width);
	      do
		{
		  carmen_get_current_point(&width, &x, &y);
		  
		  if(x >= 0 && x < map->config.x_size && y >= 0 && 
		     y < map->config.y_size)
		    {
		      w = map_x_to_map_pix(x+1) - (int)(map_x_to_map_pix(x));
		      if(w < 1)
			w = 1;
		      h = map_y_to_map_pix(y) - (int)(map_y_to_map_pix(y+1));
		      if(h < 1)
			h = 1;
		      map->map[x][y] = ink;
		      gdk_draw_rectangle(map_pixmap, drawing_gc, 1,
					 map_x_to_map_pix(x) + 1.0, 
					 map_y_to_map_pix(y+1) + 1.0, w,h);
		    }
		  
		}
	      while(carmen_get_next_point(&width));
	      carmen_get_bresenham_parameters(map_x1+1.0 * cos(theta-M_PI/ 4.0), 
					      map_y1+1.0 * sin(theta-M_PI/ 4.0), 
					      map_x2+1.0 * cos(theta-M_PI/ 4.0), 
					      map_y2+1.0 * sin(theta-M_PI/ 4.0), 
					      &width);
	      do
		{
		  carmen_get_current_point(&width, &x, &y);
		  
		  if(x >= 0 && x < map->config.x_size && y >= 0 && 
		     y < map->config.y_size)
		    {
		      w = map_x_to_map_pix(x+1) - (int)(map_x_to_map_pix(x));
		      if(w < 1)
			w = 1;
		      h = map_y_to_map_pix(y) - (int)(map_y_to_map_pix(y+1));
		      if(h < 1)
			h = 1;
		      map->map[x][y] = ink;
		      gdk_draw_rectangle(map_pixmap, drawing_gc, 1,
					 map_x_to_map_pix(x) + 1.0, 
					 map_y_to_map_pix(y+1) + 1.0, 
					 w,h);
		    }
		  
		}
	      while(carmen_get_next_point(&width));
	      
	    }
	}
      while(carmen_get_next_point(&length));
    }
  map_to_tmp();
  
}

/* records the starting point, draws a temperary line onto the drawing area
   and then calls Draw_line to draw the final line */
gint 
creating_line(int state, double pix_x, double pix_y)
{
  static double xs, ys;
  
  switch(state)
    {
    case 0:
      xs = pix_x;
      ys = pix_y;
      break;
    case 1:
      map_to_tmp();
      gdk_gc_set_foreground (drawing_gc, &yellow);
      gdk_draw_line (tmp_pixmap, drawing_gc, xs, ys, pix_x, pix_y);
      break;
    case 2:
      draw_line(xs, ys, pix_x, pix_y);
      break;
    }

  return 1;
}
/* needed for non-recursive fill and fuzzy_fill */

void 
push(carmen_stack_t ** stack, carmen_stack_t *item)
{
  item->next = *stack;
  *stack = item;
}

carmen_stack_t *
pop(carmen_stack_t ** stack)
{
  carmen_stack_t *temp;
  temp = (*stack);
  *stack = (*stack)->next;
  return temp;
}

/* fills all adjacent squares with the same probability to the ink color */
void 
fill(double under, int map_x, int map_y)
{
  carmen_stack_t *stack = NULL;
  carmen_stack_t *temp;
  int w,h;

  temp = calloc(1, sizeof(carmen_stack_t));
  carmen_test_alloc(temp);
  temp->x = map_x;
  temp->y = map_y;
  push(&stack, temp);
  while(stack != NULL)
    {
      temp = pop(&stack);
      map_x = temp->x;
      map_y = temp->y;
      free(temp);
      w = map_x_to_map_pix(map_x+1) - (int)(map_x_to_map_pix(map_x));
      if(w < 1)
	w = 1;
      h = map_y_to_map_pix(map_y) - (int)(map_y_to_map_pix(map_y+1));
      if(h < 1)
	h = 1;
      map->map[map_x][map_y] = ink;
      gdk_gc_set_foreground (drawing_gc, &color);
      gdk_draw_rectangle (map_pixmap,
			  drawing_gc,
			  TRUE,
			  map_x_to_map_pix(map_x) + 1.0, map_y_to_map_pix(map_y+1) + 1.0,
			  w, h);
      if(map_x-1 >=0 && map->map[map_x-1][map_y] == under)
	{
	  temp = calloc(1, sizeof(carmen_stack_t));
	  carmen_test_alloc(temp);
	  temp->x = map_x-1;
	  temp->y = map_y;
	  push(&stack, temp);
	}
      if(map_x+1 < map->config.x_size && map->map[map_x+1][map_y] == under)
	{
	  temp = calloc(1, sizeof(carmen_stack_t));
	  carmen_test_alloc(temp);
	  temp->x = map_x+1;
	  temp->y = map_y;
	  push(&stack, temp);
	}
      if(map_y-1 >=0 && map->map[map_x][map_y-1] == under)
	{
	  temp = calloc(1, sizeof(carmen_stack_t));
	  carmen_test_alloc(temp);
	  temp->x = map_x;
	  temp->y = map_y-1;
	  push(&stack, temp);
	}
      if(map_y+1 < map->config.y_size && map->map[map_x][map_y+1] == under)
	{
	  temp = calloc(1, sizeof(carmen_stack_t));
	  carmen_test_alloc(temp);
	  temp->x = map_x;
	  temp->y = map_y+1;
	  push(&stack, temp);
	}
    }
}

/* sets up the fill's settings and then calls fill */
gint 
create_fill(double pix_x, double pix_y)
{
  int map_x, map_y;
  map_x = pix_x_to_map(pix_x);
  map_y = pix_y_to_map(pix_y);
  
  if(map->map[map_x][map_y] == ink)
    return 1;
  fill(map->map[map_x][map_y], map_x, map_y);
  
  map_to_tmp();
  return 1;
}

/* fills all adjacent squares with probability within fuzzyness of the original 
   square to the ink color */

void 
fuzzy_fill(double under, int map_x, int map_y, int ** seen)
{
  carmen_stack_t *stack = NULL;
  carmen_stack_t *temp;
  int w,h;

  temp = calloc(1, sizeof(carmen_stack_t));
  carmen_test_alloc(temp);
  temp->x = map_x;
  temp->y = map_y;
  push(&stack, temp);
  while(stack != NULL)
    {
      temp = pop(&stack);
      map_x = temp->x;
      map_y = temp->y;
      free(temp);
      w = map_x_to_map_pix(map_x+1) - (int)(map_x_to_map_pix(map_x));
      if(w < 1)
	w = 1;
      h = map_y_to_map_pix(map_y) - (int)(map_y_to_map_pix(map_y+1));
      if(h < 1)
	h = 1;
      seen[map_x][map_y] = 1;
      map->map[map_x][map_y] = ink;
      gdk_gc_set_foreground (drawing_gc, &color);
      gdk_draw_rectangle (map_pixmap,
			  drawing_gc,
			  TRUE,
			  map_x_to_map_pix(map_x) + 1.0, map_y_to_map_pix(map_y+1) + 1.0,
			  w, h);
      if(map_x-1 >=0 && fabs(map->map[map_x-1][map_y] - under) < fuzzyness 
	 && !seen[map_x-1][map_y])
	{
	  temp = calloc(1, sizeof(carmen_stack_t));
	  carmen_test_alloc(temp);
	  temp->x = map_x-1;
	  temp->y = map_y;
	  push(&stack, temp);
	}
      if(map_x+1 < map->config.x_size && fabs(map->map[map_x+1][map_y]-under) < fuzzyness 
	 && !seen[map_x+1][map_y])
	{
	  temp = calloc(1, sizeof(carmen_stack_t));
	  carmen_test_alloc(temp);
	  temp->x = map_x+1;
	  temp->y = map_y;
	  push(&stack, temp);
	}
      if(map_y-1 >=0 && fabs(map->map[map_x][map_y-1] - under) < fuzzyness 
	 && !seen[map_x][map_y-1])
	{
	  temp = calloc(1, sizeof(carmen_stack_t));
	  carmen_test_alloc(temp);
	  temp->x = map_x;
	  temp->y = map_y-1;
	  push(&stack, temp);
	}
      if(map_y+1 < map->config.y_size && fabs(map->map[map_x][map_y+1] - under) < fuzzyness 
	 && !seen[map_x][map_y+1])
	{
	  temp = calloc(1, sizeof(carmen_stack_t));
	  carmen_test_alloc(temp);
	  temp->x = map_x;
	  temp->y = map_y+1;
	  push(&stack, temp);
	}
    }
}

/* sets up the fuzzy_fill's settings and then calls fuzzy_fill */
gint 
create_fuzzy_fill(double pix_x, double pix_y)
{
  int map_x, map_y;
  int ** seen;
  int i, j;

  seen = (int **) calloc(map->config.x_size, sizeof(int*));
  carmen_test_alloc(seen);
  for(i = 0; i < map->config.x_size; i++)
    {
      seen[i] = (int *)calloc(map->config.y_size, sizeof(int));
      carmen_test_alloc(seen[i]);
      for(j = 0; j < map->config.y_size; j++)
	seen[i][j] = 0;
    }
  map_x = pix_x_to_map(pix_x);
  map_y = pix_y_to_map(pix_y);
  
  fuzzy_fill(map->map[map_x][map_y], map_x, map_y, seen);
  map_to_tmp();

  for(i = 0; i < map->config.x_size; i++)
    free(seen[i]);
  free(seen);
  return 1;
}

/* sets ink to the probability of the square under the eye dropper */
gint 
sample(double pix_x, double pix_y)
{
  GtkAdjustment* adjustment;
  int map_x, map_y;
  map_x = pix_x_to_map(pix_x);
  map_y = pix_y_to_map(pix_y);

  if(map_y < 0 || map_y >= map->config.y_size || map_x < 0 ||map_x >= map->config.x_size)
    return 1;
  
  ink = map->map[map_x][map_y];

  if(ink < 0)
    {
      color = blue;
      gtk_label_set_text(GTK_LABEL(ink_label), "unknown");
    }
  else
    {
      adjustment = gtk_range_get_adjustment(GTK_RANGE(ink_scale));
      gtk_adjustment_set_value(adjustment,ink);
      gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");
    }
  return 1;
}

/* moves the visible canvas in the window without changeing map */
gint
move(int state, double pix_x, double pix_y)
{
  GtkAdjustment *adjustment;
  static int xs, xval, xlow, xupp, xp_s;
  static int ys, yval, ylow, yupp, yp_s;//, xm, ym;
  int x, y;
  x= pix_x_to_map(pix_x);
  y= pix_y_to_map(pix_y);
  switch(state)
    {
    case 0:
      /*     see below (*²) for comment
	      xm = pix_x;
	      ym = pix_y;      */
      xs = x;
      adjustment = gtk_scrolled_window_get_hadjustment((GtkScrolledWindow *)scrolled_window);
      xval = adjustment->value;
      xlow = adjustment->lower;
      xupp = adjustment->upper;
      xp_s = adjustment->page_size;
      ys = y;
      adjustment = gtk_scrolled_window_get_vadjustment((GtkScrolledWindow *)scrolled_window);
      yval = adjustment->value;
      ylow = adjustment->lower;
      yupp = adjustment->upper;
      yp_s = adjustment->page_size;
      move_enable = 1;
      break;
    case 1:
      if (move_enable == 1)
	{
	  //x:
	  adjustment = gtk_scrolled_window_get_hadjustment((GtkScrolledWindow *)scrolled_window);
	  if (x < xs) //move to the left
	    {
	    if (xval+xp_s+((xs-x)*mult) <= xupp) 
	      adjustment->value = xval+((xs-x)*mult);
	    else
	      adjustment->value = xupp - xp_s;
	    }
	  if (x > xs) //move to the right
	    {
	    if ((xval-((x-xs)*mult)) >= xlow)
	      adjustment->value = xval-((x-xs)*mult);
	    else
	      adjustment->value = xlow;
	    }
	  //y:
	  adjustment = gtk_scrolled_window_get_vadjustment((GtkScrolledWindow *)scrolled_window);
	  if (y > ys) //move up
	    {
	    if (yval+yp_s-((ys-y)*mult) <= yupp)
	      adjustment->value = yval-((ys-y)*mult);
	    else
	      adjustment->value = yupp - yp_s;
	    }
	  if (y < ys) //move down	    
	    {
	    if ((yval+((y-ys)*mult)) >= ylow) 
	      adjustment->value = yval+((y-ys)*mult);
	    else
	      adjustment->value = ylow;
	    }
	  /*² It would be nice to show a line to indicate the movement but this is too slow imho
	  map_to_tmp();
	  gdk_gc_set_foreground (drawing_gc, &yellow);
	  gdk_draw_line (tmp_pixmap, drawing_gc, xm, ym, pix_x, pix_y);*/
	}

      break;
    case 2:
      adjustment = gtk_scrolled_window_get_hadjustment((GtkScrolledWindow *)scrolled_window);
      gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "value_changed");
      adjustment = gtk_scrolled_window_get_vadjustment((GtkScrolledWindow *)scrolled_window);
      gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "value_changed");
      move_enable = 0;
      break;
    }
  return 1;
}

gint 
zoom_in(double pix_x, double pix_y)
{
  GtkAdjustment *adjustment;
  int map_x, map_y;
  map_x = pix_x_to_map(pix_x);
  map_y = pix_y_to_map(pix_y);
  mult *= 2;

  gtk_drawing_area_size (GTK_DRAWING_AREA (drawing_area),xend*mult,yend*mult);

  adjustment = gtk_scrolled_window_get_hadjustment((GtkScrolledWindow *)scrolled_window);
  adjustment->lower = xstart*mult;
  adjustment->upper = xend*mult;
  adjustment->page_increment = adjustment->page_size-2*mult;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");

  adjustment->value = map_x*mult-adjustment->page_size/2;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "value_changed");
  
  adjustment = gtk_scrolled_window_get_vadjustment((GtkScrolledWindow *)scrolled_window);
  adjustment->upper = (map->config.y_size - ystart)*mult;
  adjustment->lower = (map->config.y_size - yend)*mult;
  adjustment->page_increment = adjustment->page_size-2*mult;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");

  adjustment->value = (map->config.y_size-map_y)*mult-adjustment->page_size/2;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "value_changed");
 
  //rulers will have to be handled here

  if(map_pixmap)
    {
      gdk_pixmap_unref(map_pixmap);
      gdk_pixmap_unref(tmp_pixmap);
    }
  map_pixmap = NULL;
  tmp_pixmap = NULL;
  
  redraw();
  
  return 1;
}

gint 
zoom_out(double pix_x, double pix_y)
{
  GtkAdjustment *adjustment;
  int map_x, map_y;
  map_x = pix_x_to_map(pix_x);
  map_y = pix_y_to_map(pix_y);

  mult /= 2;

  gtk_drawing_area_size (GTK_DRAWING_AREA (drawing_area),xend*mult,yend*mult);

  adjustment = gtk_scrolled_window_get_hadjustment((GtkScrolledWindow *)scrolled_window);
  adjustment->lower = xstart*mult;
  adjustment->upper = xend*mult;
  adjustment->page_increment = adjustment->page_size-2*mult;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");

  adjustment->value = map_x*mult-adjustment->page_size/2;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "value_changed");

  adjustment = gtk_scrolled_window_get_vadjustment((GtkScrolledWindow *)scrolled_window);
  adjustment->upper = (map->config.y_size - ystart)*mult;
  adjustment->lower = (map->config.y_size - yend)*mult;
  adjustment->page_increment = adjustment->page_size-2*mult;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "changed");
  
  adjustment->value = (map->config.y_size - map_y)*mult-adjustment->page_size/2;
  gtk_signal_emit_by_name (GTK_OBJECT (adjustment), "value_changed");
  
  //rulers will have to be handled here 
  //  gtk_ruler_set_range (GTK_RULER (hrule), xstart*mult, xend*mult, xstart*mult, xend*mult);

  if(map_pixmap)
    {
      gdk_pixmap_unref(map_pixmap);
      gdk_pixmap_unref(tmp_pixmap);
    }
  map_pixmap = NULL;
  tmp_pixmap = NULL;
  
  redraw();

  return 1;
}

