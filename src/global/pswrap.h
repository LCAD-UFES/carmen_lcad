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

/***********************************************************
 *
 * PSWRAP.H
 *
 * Postscript output library 
 * Copyright (c) 2002 Mike Montemerlo
 *
 ***********************************************************/


/** @addtogroup global libpswrap **/
// @{

/** \file pswrap.h
 * \brief Library for writing postscript files.
 *
 * Library for writing postscript files.
 **/



#ifndef CARMEN_PSWRAP_H
#define CARMEN_PSWRAP_H

#ifdef __cplusplus
extern "C" {
#endif

#define      CARMEN_GENERATE_PS         0
#define      CARMEN_GENERATE_EPS        1

#define      CARMEN_PS_BUTTCAP          0
#define      CARMEN_PS_ROUNDCAP         1
#define      CARMEN_PS_PROJECTINGCAP    2

#define      CARMEN_PS_MITERJOIN        0
#define      CARMEN_PS_ROUNDJOIN        1
#define      CARMEN_PS_BEVELJOIN        2

typedef struct {
  FILE *fp;
  float width, height;
  int border, eps;
  int page_count;
} carmen_ps_doc_t, *carmen_ps_doc_p;

carmen_ps_doc_p carmen_ps_open(char *filename, float width, float height, int eps);
void carmen_ps_close(carmen_ps_doc_p doc);
void carmen_ps_comment(carmen_ps_doc_p doc, char *comment);
void carmen_ps_next_page(carmen_ps_doc_p doc);
void carmen_ps_set_color(carmen_ps_doc_p doc, unsigned char r, unsigned char g,
		  unsigned char b);
void carmen_ps_set_gray(carmen_ps_doc_p doc, unsigned char level);
void carmen_ps_set_linewidth(carmen_ps_doc_p doc, float w);
void carmen_ps_set_jointype(carmen_ps_doc_p doc, int join);
void carmen_ps_set_captype(carmen_ps_doc_p doc, int cap);
void carmen_ps_set_dash(carmen_ps_doc_p doc, int length);
void carmen_ps_set_font(carmen_ps_doc_p doc, char *fontname, int size);

void carmen_ps_draw_point(carmen_ps_doc_p doc, float x_1, float y_1);
void carmen_ps_draw_points(carmen_ps_doc_p doc, float *x_1, float *y_1, int n);
void carmen_ps_draw_line(carmen_ps_doc_p doc, float x_1, float y_1, float x_2, float y_2);
void carmen_ps_draw_x(carmen_ps_doc_p doc, float x_1, float y_1, float r);
void carmen_ps_draw_poly(carmen_ps_doc_p doc, int filled, float *x_1, 
			 float *y_1, int n, int closed);
void carmen_ps_draw_circle(carmen_ps_doc_p doc, int filled, float x_1, 
			   float y_1, float r);
void carmen_ps_draw_ellipse(carmen_ps_doc_p doc, int filled, float x_1, 
			    float y_1, float x_var, float xy_cov, float y_var, 
			    float k);
void carmen_ps_draw_gaussian(carmen_ps_doc_p doc, float x, float y, 
			     float x_var, float xy_cov, float y_var, float k);
void carmen_ps_draw_rectangle(carmen_ps_doc_p doc, int filled, float x_1, 
			      float y_1, float x_2, float y_2);
void carmen_ps_draw_arc(carmen_ps_doc_p doc, float x_1, float y_1, float r, 
			float start_angle, float delta);
void carmen_ps_draw_wedge(carmen_ps_doc_p doc, int filled, float x_1, 
			  float y_1, float r, float start_angle, float delta);
void carmen_ps_draw_text(carmen_ps_doc_p doc, char *str, float x_1, float y_1);
void carmen_ps_draw_text_landscape(carmen_ps_doc_p doc, char *str, float x_1, 
				   float y_1);
void carmen_ps_draw_image(carmen_ps_doc_p doc, float dest_x, float dest_y, 
			  float dest_width, float dest_height, char *src_image,
			  int src_width, int src_height);
void carmen_ps_draw_transformed_image(carmen_ps_doc_p doc, char *srt_image, 
				      int src_width, int src_height, 
				      float dest_x, float dest_y,
				      float dest_theta, float dest_scale);

#ifdef __cplusplus
}
#endif 

#endif
// @}
