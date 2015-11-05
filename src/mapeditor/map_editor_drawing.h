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
#ifndef CARMEN_MAP_EDITOR_DRAWING_H
#define CARMEN_MAP_EDITOR_DRAWING_H

#ifdef __cplusplus
extern "C" {
#endif

/* set the bitmap for the cursor */
gint set_point(void);
gint set_cross(void);
gint set_pour(void);
gint set_dropper(void);
/* set the ink */
gint set_ink(GtkAdjustment *adj);
gint set_unknown(void);
gint set_offlimits(void);
/* set the fuzynes for fuyzzy fill */
gint set_fuzzyness(GtkAdjustment *adj);
/* set which tool is being used */
gint set_brush(void);
gint set_rectangle(void);
gint set_crop(void);
gint set_line(void);
gint set_fill(void);
gint set_fuzzy_fill(void);
gint set_sample(void);
gint set_zoom(void);
gint set_mover(void);
/* set filled or not filled */
gint set_filled(void);
gint set_not_filled(void);
/* change line size */
gint line_incr(void);
gint line_decr(void);
/* change brush size */
gint brush_decr(void);
gint brush_incr(void);

/* draw with the brush */
void draw_brush( GtkWidget *widget, gdouble pix_x, gdouble pix_y);
/* start a rectangle.
   state of 0 corresponds to button_press_event
   state of 1 corresponds to motion_notify_event (draws a temperary rectangle) 
   state of 2 corresponds to button_release event (draws the final rectangle)
*/
void creating_rectangle(GtkWidget *widget, int state, double pix_x, double pix_y);
void cropping(GtkWidget *widget, int state, double pix_x, double pix_y);
/* similar to creating rectangle */
gint creating_line(int state, double pix_x, double pix_y);
/* starts fill */
gint create_fill(double pix_x, double pix_y);
/* starts fuzzy fill */
gint create_fuzzy_fill(double pix_x, double pix_y);
/* sets the ink to the probability at the sampled point */
gint sample(double pix_x, double pix_y);
/* zoom in */
gint zoom_in(double pix_x, double pix_y);
/* zoom out */
gint zoom_out(double pix_x, double pix_y);
/* move map */
  gint move(int state, double pix_x, double pix_y);

#ifdef __cplusplus
}
#endif

#endif
