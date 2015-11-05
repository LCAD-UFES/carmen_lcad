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

/****************************************************
 *
 * ARTWRAP.H
 *
 * Wrapper around libart function calls that makes
 * using libart more like gtk.
 *
 * written by Mike Montemerlo (mmde@cs.cmu.edu)
 * Carngie Mellon Univeristy
 *
 ****************************************************/


/** @addtogroup global libartwrap **/
// @{

/** \file artwrap.h
 * \brief Wrapper around libart function calls that makes
 * using libart more like gtk.
 *
 * Wrapper around libart function calls that makes
 * using libart more like gtk.
 **/


#ifndef CARMEN_ARTWRAP_H
#define CARMEN_ARTWRAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <libart_lgpl/art_misc.h>
#include <libart_lgpl/art_vpath.h>
#include <libart_lgpl/art_svp.h>
#include <libart_lgpl/art_svp_vpath.h>
#include <libart_lgpl/art_gray_svp.h>
#include <libart_lgpl/art_rgb_svp.h>
#include <libart_lgpl/art_svp_vpath_stroke.h>
#include <libart_lgpl/art_svp_ops.h>
#include <libart_lgpl/art_affine.h>
#include <libart_lgpl/art_rgb.h>
#include <libart_lgpl/art_rgb_affine.h>
#include <libart_lgpl/art_rgb_bitmap_affine.h>
#include <libart_lgpl/art_rgb_rgba_affine.h>
#include <libart_lgpl/art_alphagamma.h>
#include <libart_lgpl/art_svp_point.h>
#include <libart_lgpl/art_rect_svp.h>
#include <libart_lgpl/art_vpath_dash.h>
#include <libart_lgpl/art_render.h>
#include <libart_lgpl/art_render_gradient.h>
#include <libart_lgpl/art_render_svp.h>
#include <libart_lgpl/art_uta.h>
#include <libart_lgpl/art_uta_vpath.h>
#include <libart_lgpl/art_rect_uta.h>

typedef struct {
  int width, height;
  art_u8 *buffer;
} art_buffer_t, *art_buffer_p;

typedef struct {
  art_u32 current_color;
  int current_linewidth;
  int current_miterlimit;
  int current_jointype;
  int current_captype;
  float current_flatness;
  int current_dash_on, current_dash_off;
} art_context_t, *art_context_p;

#define art_red     0xff0000ff
#define art_blue    0x0000ffff
#define art_green   0x00ff00ff
#define art_white   0xffffffff
#define art_black   0x000000ff
#define art_yellow  0xffff00ff
#define art_orange  0xff6347ff
#define art_grey    0x808080ff

art_buffer_p art_new_buffer(int width, int height);

art_context_p art_new_context(void);

void art_set_color_rgba(art_context_p context, unsigned char r, 
			unsigned char g, unsigned char b, unsigned char a);

void art_set_color(art_context_p context, long int color);

void art_set_linewidth(art_context_p context, int width);

void art_set_miterlimit(art_context_p context, int limit);

void art_set_jointype(art_context_p context, int jointype);

void art_set_captype(art_context_p context, int captype);
     
void art_set_dash(art_context_p context, int dash_on, int dash_off);

void art_clear(art_buffer_p buffer, art_context_p context);

void art_draw_point(art_buffer_p buffer, art_context_p context,
		    float x_1, float y_1);

void art_draw_points(art_buffer_p buffer, art_context_p context,
		     float *x_1, float *y_1, int num);

void art_draw_line(art_buffer_p buffer, art_context_p context, 
		   float x_1, float y_1, float x_2, float y_2);

void art_draw_x(art_buffer_p buffer, art_context_p context, 
		float x, float y, float r);

void art_draw_poly(art_buffer_p buffer, art_context_p context, int filled,
		   float *x, float *y, int n, int closed);

void art_draw_circle(art_buffer_p buffer, art_context_p context, int filled,
		     float x, float y, float r);

void art_draw_ellipse(art_buffer_p buffer, art_context_p context, int filled,
		      float x, float y, float x_var, 
		      float xy_cov, float y_var, float k);

void art_draw_gaussian(art_buffer_p buffer, art_context_p context,
		       float x, float y, float x_var, 
		       float xy_cov, float y_var, float k);

void art_draw_gaussian_pca(art_buffer_p buffer, art_context_p context,
		       float x, float y, float x_var, 
			   float xy_cov, float y_var, float k);

void art_draw_rectangle(art_buffer_p buffer, art_context_p context, int filled,
			float x_1, float y_1, float x_2, float y_2);

void art_draw_arc(art_buffer_p buffer, art_context_p context,
		  float x, float y, float r, float a1, float deltaa);

void art_draw_wedge(art_buffer_p buffer, art_context_p context, int filled,
		    float x, float y, float r, float a1, float deltaa);

void art_draw_rgb_image(art_buffer_p dest_buffer, int dest_x, int dest_y,
			int dest_width, int dest_height, 
			unsigned char *src_buffer, int src_width,
			int src_height);

void art_draw_transformed_rgb_image(art_buffer_p dest_buffer, 
				    unsigned char *src_buffer, int src_width,
				    int src_height, int dest_x, int dest_y,
				    float angle, float scale);

void art_draw_transformed_rgba_image(art_buffer_p dest_buffer, 
				     unsigned char *src_buffer, int src_width,
				     int src_height, int dest_x, int dest_y,
				     float angle, float scale);

void art_draw_rgba_image(art_buffer_p dest_buffer, int dest_x, int dest_y,
			 int dest_width, int dest_height, 
			 unsigned char *src_buffer, int src_width,
			 int src_height);

int art_write_buffer_ppm(art_buffer_p buffer, char *filename);

int art_write_buffer_png(art_buffer_p buffer, char *filename);

#ifdef __cplusplus
}
#endif

#endif


// @}
