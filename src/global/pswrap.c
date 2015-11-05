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
 * PSWRAP.C
 *
 * Postscript output library 
 * Copyright (c) 2002 Mike Montemerlo
 *
 ***********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "pswrap.h"

#define MAX_POLY_POINTS 30

carmen_ps_doc_p carmen_ps_open(char *filename, float width, float height, int eps)
{
  carmen_ps_doc_p doc;
  float lm, tm;

  doc = (carmen_ps_doc_p)calloc(1, sizeof(carmen_ps_doc_t));
  /* carmen_check_alloc */
  if(doc == NULL)
    return NULL;
  doc->fp = fopen(filename, "w");
  if(doc->fp != NULL) {
    doc->width = width;
    doc->height = height;
    doc->eps = eps;
    doc->page_count = 1;
    if(height == 14) {
      lm = (8.5 - width) / 2.0;
      tm = (14 - height) / 2.0;
    }
    else {
      lm = (8.5 - width) / 2.0;
      tm = (11 - height) / 2.0;
    }
    if(doc->eps)
      fprintf(doc->fp, "%%!PS-Adobe-3.0 EPSF-2.0\n");
    else 
      fprintf(doc->fp, "%%!PS-Adobe-2.0\n");
    fprintf(doc->fp, "%%%%BoundingBox: %d %d %d %d\n", 
	    (int)(lm * 72), (int)(tm * 72), (int)((lm + width) * 72),
	    (int)((tm + height) * 72));
    if(!doc->eps)
      fprintf(doc->fp, "%%%%Pages: (atend)\n");
    fprintf(doc->fp, "%%%%EndComments\n");

    fprintf(doc->fp, "/DeviceRGB setcolorspace\n");
    fprintf(doc->fp, "/inch {72 mul} def\n");

    if(!doc->eps)
      fprintf(doc->fp, "%%%%Page: 1 1\n");
    fprintf(doc->fp, "%f inch %f inch translate\n", lm, tm);
    fprintf(doc->fp, "newpath\n");
    fprintf(doc->fp, "0 0 moveto\n");
    fprintf(doc->fp, "%f inch 0 lineto\n", width);
    fprintf(doc->fp, "%f inch %f inch lineto\n", width, height);
    fprintf(doc->fp, "0 %f inch lineto\n", height);
    fprintf(doc->fp, "closepath\nclip\n");
  }
  return doc;
}

void carmen_ps_close(carmen_ps_doc_p doc)
{
  if(!doc->eps) {
    fprintf(doc->fp, "%%%%Pages: %d\n", doc->page_count);
    fprintf(doc->fp, "showpage\n");
  }
  else
    fprintf(doc->fp, "%%%%EOF\n");
  fclose(doc->fp);
  free(doc);
}

void carmen_ps_comment(carmen_ps_doc_p doc, char *comment)
{
  char temp[100];
  int i;

  strncpy(temp, comment, 100);
  for(i = 0; i < (signed int)strlen(temp); i++)
    if(temp[i] == '\n')
      temp[i] = ' ';
  fprintf(doc->fp, "%%%s\n", temp);
}

void carmen_ps_next_page(carmen_ps_doc_p doc)
{
  float height, width, lm, tm;

  if(!doc->eps) {
    width = doc->width;
    height = doc->height;
    lm = (8.5 - width) / 2.0;
    tm = 11 - height - 1;

    fprintf(doc->fp, "showpage\n");
    (doc->page_count)++;
    fprintf(doc->fp, "%%%%Page: %d %d\n", doc->page_count, doc->page_count);
    fprintf(doc->fp, "%f inch %f inch translate\n", lm, tm);
    fprintf(doc->fp, "newpath\n");
    fprintf(doc->fp, "0 0 moveto\n");
    fprintf(doc->fp, "%f inch 0 lineto\n", width);
    fprintf(doc->fp, "%f inch %f inch lineto\n", width, height);
    fprintf(doc->fp, "0 %f inch lineto\n", height);
    fprintf(doc->fp, "closepath\nclip\n");
  }
}

void carmen_ps_set_color(carmen_ps_doc_p doc, unsigned char r, unsigned char g,
		  unsigned char b)
{
  fprintf(doc->fp, "%f %f %f setrgbcolor\n", r / 255.0, g / 255.0, b / 255.0);
}

void carmen_ps_set_gray(carmen_ps_doc_p doc, unsigned char level)
{
  fprintf(doc->fp, "%f setgray\n", level / 255.0);
}

void carmen_ps_set_linewidth(carmen_ps_doc_p doc, float w)
{
  fprintf(doc->fp, "%f setlinewidth\n", w);
}

void carmen_ps_set_jointype(carmen_ps_doc_p doc, int join)
{
  fprintf(doc->fp, "%d setlinejoin\n", join);
}

void carmen_ps_set_captype(carmen_ps_doc_p doc, int cap)
{
  fprintf(doc->fp, "%d setlinecap\n", cap);
}

void carmen_ps_set_dash(carmen_ps_doc_p doc, int length)
{
  if(length == 0)
    fprintf(doc->fp, "[] 0 setdash\n");
  else
    fprintf(doc->fp, "[%d] 0 setdash\n", length);
}

void carmen_ps_set_font(carmen_ps_doc_p doc, char *fontname, int size)
{
  fprintf(doc->fp, "/%s findfont %d scalefont setfont\n", fontname, size);
}

void carmen_ps_draw_point(carmen_ps_doc_p doc, float x_1, float y_1)
{
  fprintf(doc->fp, "newpath\n");
  fprintf(doc->fp, "%f inch %f inch %f inch 0 360 arc\n", x_1, y_1, 0.01);
  fprintf(doc->fp, "fill\n");
}

void carmen_ps_draw_points(carmen_ps_doc_p doc, float *x_1, float *y_1, int n)
{
  int i;

  for(i = 0; i < n; i++)
    carmen_ps_draw_point(doc, x_1[i], y_1[i]);
}

void carmen_ps_draw_line(carmen_ps_doc_p doc, float x_1, float y_1, float x_2, float y_2)
{
  fprintf(doc->fp, "newpath\n");
  fprintf(doc->fp, "%f inch %f inch moveto\n", x_1, y_1);
  fprintf(doc->fp, "%f inch %f inch lineto\n", x_2, y_2);
  fprintf(doc->fp, "stroke\n");
}

void carmen_ps_draw_x(carmen_ps_doc_p doc, float x_1, float y_1, float r)
{
  carmen_ps_draw_line(doc, x_1 - r, y_1 - r, x_1 + r, y_1 + r);
  carmen_ps_draw_line(doc, x_1 - r, y_1 + r, x_1 + r, y_1 - r);
}

void carmen_ps_draw_poly(carmen_ps_doc_p doc, int filled, float *x_1, float *y_1, int n,
		  int closed)
{
  int i;

  fprintf(doc->fp, "newpath\n");
  fprintf(doc->fp, "%f inch %f inch moveto\n", x_1[0], y_1[0]);
  for(i = 1; i < n; i++)
    fprintf(doc->fp, "%f inch %f inch lineto\n", x_1[i], y_1[i]);
  if(closed)
    fprintf(doc->fp, "closepath\n");
  if(filled)
    fprintf(doc->fp, "fill\n");
  else
    fprintf(doc->fp, "stroke\n");
}

void carmen_ps_draw_circle(carmen_ps_doc_p doc, int filled, float x_1, float y_1, float r)
{
  fprintf(doc->fp, "newpath\n");
  fprintf(doc->fp, "%f inch %f inch %f inch 0 360 arc\n", x_1, y_1, r);
  if(filled)
    fprintf(doc->fp, "fill\n");
  else
    fprintf(doc->fp, "stroke\n");
}

void carmen_ps_draw_ellipse(carmen_ps_doc_p doc, int filled, float x_1, float y_1, 
		     float x_var, float xy_cov, float y_var, float k)
{
  float poly_x[MAX_POLY_POINTS], poly_y[MAX_POLY_POINTS], l11, l21, l22;
  int i;
  
  l11 = sqrt(x_var);
  l21 = xy_cov / l11;
  l22 = sqrt(y_var - l21 * l21);
  for(i = 0; i < MAX_POLY_POINTS; i++) {
    float t = i / (float)MAX_POLY_POINTS * 2 * M_PI, xt, yt;
    xt = cos(t);    yt = sin(t);
    poly_x[i] = x_1 + l11 * xt * k;
    poly_y[i] = y_1 + (l21 * xt + l22 * yt) * k;
  }
  carmen_ps_draw_poly(doc, filled, poly_x, poly_y, MAX_POLY_POINTS, 1);
}


void carmen_ps_draw_gaussian(carmen_ps_doc_p doc, float x_1, float y_1, float x_var, 
		      float xy_cov, float y_var, float k)
{
  float poly_x[MAX_POLY_POINTS], poly_y[MAX_POLY_POINTS];
  float len, discriminant, eigval1, eigval2;
  float eigvec1x, eigvec1y, eigvec2x, eigvec2y;
  float ex1, ey1, ex2, ey2;
  int i;
  
  /* check for special case of axis-aligned */
  if (fabs(xy_cov) < (fabs(x_var) + fabs(y_var) + 1e-4) * 1e-4) {
    eigval1 = x_var;
    eigval2 = y_var;
    eigvec1x = 1.;
    eigvec1y = 0.;
    eigvec2x = 0.;
    eigvec2y = 1.;
  }
  else {
    /* compute axes and scales of ellipse */
    discriminant = sqrt(4 * xy_cov * xy_cov + 
			(x_var - y_var) * (x_var - y_var));
    eigval1 = .5 * (x_var + y_var - discriminant);
    eigval2 = .5 * (x_var + y_var + discriminant);
    eigvec1x = (x_var - y_var - discriminant) / (2.0 * xy_cov);
    eigvec1y = 1.0;
    eigvec2x = (x_var - y_var + discriminant) / (2.0 * xy_cov);
    eigvec2y = 1.0;
    /* normalize eigenvectors */
    len = sqrt(eigvec1x * eigvec1x + 1.0);
    eigvec1x /= len;
    eigvec1y /= len;
    len = sqrt(eigvec2x * eigvec2x + 1.0);
    eigvec2x /= len;
    eigvec2y /= len;
  }

  /* take square root of eigenvalues and scale -- once this is
     done, eigvecs are unit vectors along axes and eigvals are
     corresponding radii */
  if (eigval1 < 0 || eigval2 < 0) {
    carmen_ps_draw_point(doc, x_1, y_1);
    return;
  }
  eigval1 = sqrt(eigval1) * k;
  eigval2 = sqrt(eigval2) * k;
  
  /* compute points around edge of ellipse */
  for (i = 0; i < MAX_POLY_POINTS; i++) {
    float theta = M_PI * (-1.0 + 2.0 * i / (float)MAX_POLY_POINTS);
    float xi = cos(theta) * eigval1;
    float yi = sin(theta) * eigval2;
    poly_x[i] = xi * eigvec1x + yi * eigvec2x + x_1;
    poly_y[i] = xi * eigvec1y + yi * eigvec2y + y_1;
  }

  /* finally we can draw it */
  carmen_ps_draw_poly(doc, 0, poly_x, poly_y, MAX_POLY_POINTS, 1);
  ex1 = x_1 + eigval1 * eigvec1x;
  ey1 = y_1 + eigval1 * eigvec1y;
  ex2 = x_1 - eigval1 * eigvec1x;
  ey2 = y_1 - eigval1 * eigvec1y;
  carmen_ps_draw_line(doc, ex1, ey1, ex2, ey2);
  ex1 = x_1 + eigval2 * eigvec2x;
  ey1 = y_1 + eigval2 * eigvec2y;
  ex2 = x_1 - eigval2 * eigvec2x;
  ey2 = y_1 - eigval2 * eigvec2y;
  carmen_ps_draw_line(doc, ex1, ey1, ex2, ey2);
}

void carmen_ps_draw_rectangle(carmen_ps_doc_p doc, int filled, float x_1, float y_1,
		       float x_2, float y_2)
{
  float xp[4] = {x_1, x_2, x_2, x_1}, yp[4] = {y_1, y_1, y_2, y_2};
  
  carmen_ps_draw_poly(doc, filled, xp, yp, 4, 1);
}

void carmen_ps_draw_arc(carmen_ps_doc_p doc, float x_1, float y_1, float r, 
		 float start_angle, float delta)
{
  fprintf(doc->fp, "newpath\n");
  fprintf(doc->fp, "%f inch %f inch %f inch %f %f arc\n", x_1, y_1, r,
	  start_angle * 180 / M_PI, (start_angle + delta) * 180 / M_PI);
  fprintf(doc->fp, "stroke\n");
}

void carmen_ps_draw_wedge(carmen_ps_doc_p doc, int filled, float x_1, float y_1, float r,
		   float start_angle, float delta)
{
  fprintf(doc->fp, "newpath\n");
  fprintf(doc->fp, "%f inch %f inch %f inch %f %f arc\n", x_1, y_1, r,
	  start_angle * 180 / M_PI, (start_angle + delta) * 180 / M_PI);
  fprintf(doc->fp, "%f inch %f inch lineto\n", x_1, y_1);
  fprintf(doc->fp, "%f inch %f inch lineto\n", 
	  x_1 + cos(start_angle) * r, y_1 + sin(start_angle) * r);
  if(filled)
    fprintf(doc->fp, "fill\n");
  else
    fprintf(doc->fp, "stroke\n");
}

void carmen_ps_draw_text_landscape(carmen_ps_doc_p doc, char *str, float x_1, float y_1)
{
  fprintf(doc->fp, "gsave\n");
  fprintf(doc->fp, "%f inch %f inch translate\n", x_1, y_1);
  fprintf(doc->fp, "%f rotate\n", -90.0);
  fprintf(doc->fp, "0 inch 0 inch moveto\n");
  fprintf(doc->fp, "(%s) show\n", str);
  fprintf(doc->fp, "\ngrestore\n");
}

void carmen_ps_draw_text(carmen_ps_doc_p doc, char *str, float x_1, float y_1)
{
  fprintf(doc->fp, "%f inch %f inch moveto\n", x_1, y_1);
  fprintf(doc->fp, "(%s) show\n", str);
}

void carmen_ps_draw_image(carmen_ps_doc_p doc, float dest_x, float dest_y, float dest_width,
		   float dest_height, char *src_image, int src_width,
		   int src_height)
{
  int i;
  
  fprintf(doc->fp, "gsave\n");
  fprintf(doc->fp, "%f inch %f inch translate\n", dest_x, 
	  dest_y + dest_height);
  fprintf(doc->fp, "%f inch %f inch scale\n", dest_width, -dest_height);
  fprintf(doc->fp, "/picstr %d string def\n", src_width);
  fprintf(doc->fp, "%d %d 8 [%d 0 0 %d 0 0] {currentfile picstr readhexstring pop} false 3 colorimage\n", src_width, src_height, src_width, src_height);

  for(i = 0; i < src_width * src_height; i++) {
    fprintf(doc->fp, "%02x%02x%02x", 
	    (unsigned char)src_image[i * 3],
	    (unsigned char)src_image[i * 3 + 1],
	    (unsigned char)src_image[i * 3 + 2]);
    if(i % 50 == 0)
      fprintf(doc->fp, "\n");
  }
  fprintf(doc->fp, "\ngrestore\n");
}

void carmen_ps_draw_transformed_image(carmen_ps_doc_p doc, char *src_image, int src_width,
			       int src_height, float dest_x, float dest_y,
			       float dest_theta, float dest_scale)
{
  int i;
  
  fprintf(doc->fp, "gsave\n");
  fprintf(doc->fp, "%f inch %f inch translate\n", dest_x, dest_y);
  fprintf(doc->fp, "%f rotate\n", dest_theta * 180.0 / M_PI);
  fprintf(doc->fp, "%f inch %f inch scale\n", doc->width * dest_scale,
	  doc->height * dest_scale);
  fprintf(doc->fp, "/picstr %d string def\n", src_width);
  fprintf(doc->fp, "%d %d 8 [%d 0 0 %d 0 %d] {currentfile picstr readhexstring pop} false 3 colorimage\n", src_width, src_height, src_width,
	  -src_height, src_height);

  for(i = 0; i < src_width * src_height; i++) {
    fprintf(doc->fp, "%02x%02x%02x", 
	    (unsigned char)src_image[i * 3],
	    (unsigned char)src_image[i * 3 + 1],
	    (unsigned char)src_image[i * 3 + 2]);
    if(i % 50 == 0)
      fprintf(doc->fp, "\n");
  }
  fprintf(doc->fp, "\ngrestore\n");
}


