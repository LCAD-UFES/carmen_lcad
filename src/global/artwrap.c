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
 * ARTWRAP.C
 *
 * Wrapper around libart function calls that makes
 * using libart more like gtk.
 *
 * written by Mike Montemerlo (mmde@cs.cmu.edu)
 * Carngie Mellon Univeristy
 *
 ****************************************************/

#include "artwrap.h"
#include <carmen/carmen.h>

#define MIN_POLY_POINTS 10
#define MAX_POLY_POINTS 30

art_buffer_p art_new_buffer(int width, int height)
{
  art_buffer_p buffer;

  buffer = art_new(art_buffer_t, 1);
  carmen_test_alloc(buffer);
  buffer->buffer = art_new(art_u8, width * height * 3);
  carmen_test_alloc(buffer->buffer);
  buffer->width = width;
  buffer->height = height;
  return buffer;
}

art_context_p art_new_context(void)
{
  art_context_p context;

  context = art_new(art_context_t, 1);
  carmen_test_alloc(context);
  context->current_color = 0xffffffff;
  context->current_linewidth = 1;
  context->current_miterlimit = 1;
  context->current_jointype = ART_PATH_STROKE_JOIN_MITER;
  context->current_captype = ART_PATH_STROKE_CAP_BUTT;
  context->current_flatness = 0.5;
  context->current_dash_on = -1;
  context->current_dash_off = -1;
  return context;
}

void art_set_color_rgba(art_context_p context, unsigned char r, 
			unsigned char g, unsigned char b, unsigned char a)
{
  context->current_color = (r << 24) | (g << 16) | (b << 8) | (a);
}

void art_set_color(art_context_p context, long int color)
{
  context->current_color = color;
}

void art_set_linewidth(art_context_p context, int width)
{
  context->current_linewidth = width;
}

void art_set_miterlimit(art_context_p context, int limit)
{
  context->current_miterlimit = limit;
}

void art_set_jointype(art_context_p context, int jointype)
{
  context->current_jointype = jointype;
}

void art_set_captype(art_context_p context, int captype)
{
  context->current_captype = captype;
}

void art_set_dash(art_context_p context, int dash_on, int dash_off)
{
  context->current_dash_on = dash_on;
  context->current_dash_off = dash_off;
}

void art_clear(art_buffer_p buffer, art_context_p context)
{
  unsigned char r = ((context->current_color & 0xff000000) >> 24);
  unsigned char g = ((context->current_color & 0xff0000) >> 16);
  unsigned char b = ((context->current_color & 0xff00) >> 8);
  unsigned char a = (context->current_color & 0xff);
  art_rgb_run_alpha(buffer->buffer, r, g, b, a, 
		    buffer->width * buffer->height);
}

void art_draw_poly(art_buffer_p buffer, art_context_p context, int filled,
		   float *x, float *y, int n, int closed)
{
  ArtVpath *vec, *vec2;
  ArtSVP *svp;
  double dash_data[2];
  ArtVpathDash dash;
  ArtDRect drect;
  ArtIRect irect;
  int i, mark = 0;

  vec = art_new(ArtVpath, n + 1 + closed);
  for(i = 0; i < n; i++) {
    vec[mark].code = i ? ART_LINETO : ART_MOVETO;
    if(i == 0 || i == n - 1 || hypot(x[i] - vec[mark - 1].x,
				     (buffer->height - y[i]) - 
				     vec[mark - 1].y) > 1.0) {
      vec[mark].x = x[i];
      vec[mark].y = buffer->height - y[i];
      mark++;
    }
  }
  n = mark;
  if(closed) {
    vec[n].code = ART_LINETO;
    vec[n].x = vec[0].x;
    vec[n].y = vec[0].y;
  }
  vec[n + closed].code = ART_END;
  vec[n + closed].x = 0;
  vec[n + closed].y = 0;
  if(context->current_dash_on > 0) {
    dash.offset = 0;
    dash_data[0] = context->current_dash_on;
    dash_data[1] = context->current_dash_off;
    dash.n_dash = 2;
    dash.dash = dash_data;
    vec2 = art_vpath_dash(vec, &dash);
    art_free(vec);
    vec = vec2;
  }

  if(filled)
    svp = art_svp_from_vpath(vec);
  else
    svp = art_svp_vpath_stroke(vec, context->current_jointype, 
			       context->current_captype,
			       context->current_linewidth,
			       context->current_miterlimit, 
			       context->current_flatness);
  art_free(vec);

  art_drect_svp(&drect, svp);
  art_drect_to_irect(&irect, &drect);
  if(irect.x1 > buffer->width)
    irect.x1 = buffer->width;
  if(irect.y1 > buffer->height)
    irect.y1 = buffer->height;
  if(irect.x0 < 0)
    irect.x0 = 0;
  if(irect.y0 < 0)
    irect.y0 = 0;
  art_rgb_svp_alpha(svp, irect.x0, irect.y0, irect.x1, irect.y1, 
		    context->current_color,
		    buffer->buffer + (irect.y0 * buffer->width + irect.x0) * 3,
		    buffer->width * 3, NULL);
  art_svp_free(svp);
}

void art_draw_point(art_buffer_p buffer, art_context_p context,
		    float x_1, float y_1)
{
  if(x_1 >= 0 && y_1 >= 0 && x_1 < buffer->width && y_1 < buffer->height)
    art_rgb_run_alpha(buffer->buffer + 
		      ((buffer->height - 1 - (int)y_1) * buffer->width + 
		       (int)x_1) * 3,
		      (context->current_color & 0xff000000) >> 24,
		      (context->current_color & 0x00ff0000) >> 16,
		      (context->current_color & 0x0000ff00) >> 8,
		      context->current_color & 0x000000ff, 1);
}

void art_draw_points(art_buffer_p buffer, art_context_p context,
		     float *x_1, float *y_1, int num)
{
  int i;
  
  for(i = 0; i < num; i++)
    if(x_1[i] >= 0 && y_1[i] >= 0 && x_1[i] < buffer->width && 
       y_1[i] < buffer->height)
      art_rgb_run_alpha(buffer->buffer + 
			((buffer->height - 1 - (int)y_1[i]) * buffer->width + 
			 (int)x_1[i]) * 3,
			(context->current_color & 0xff000000) >> 24,
			(context->current_color & 0x00ff0000) >> 16,
			(context->current_color & 0x0000ff00) >> 8,
			context->current_color & 0x000000ff, 1);
}

void art_draw_line(art_buffer_p buffer, art_context_p context, 
		   float x_1, float y_1, float x_2, float y_2)
{
  float x[2], y[2];
  
  x[0] = x_1; y[0] = y_1; x[1] = x_2; y[1] = y_2;
  art_draw_poly(buffer, context, 0, x, y, 2, 0);
}

void art_draw_x(art_buffer_p buffer, art_context_p context, 
		float x, float y, float r)
{
  art_draw_line(buffer, context, 
		x - 0.707 * r, y - 0.707 * r,
		x + 0.707 * r, y + 0.707 * r);
  art_draw_line(buffer, context, 
		x - 0.707 * r, y + 0.707 * r,
		x + 0.707 * r, y - 0.707 * r);
}

void art_draw_circle(art_buffer_p buffer, art_context_p context, int filled,
		     float x, float y, float r)
{
  int i, n = r / 20.0 * MAX_POLY_POINTS;
  float t, poly_x[MAX_POLY_POINTS], poly_y[MAX_POLY_POINTS];

  if(n > MAX_POLY_POINTS)
    n = MAX_POLY_POINTS;
  else if(n < MIN_POLY_POINTS)
    n = MIN_POLY_POINTS;
  for(i = 0; i < n; i++) {
    t = i / (float)n * 2 * M_PI;
    poly_x[i] = x + r * cos(t);
    poly_y[i] = y + r * sin(t);
  }
  art_draw_poly(buffer, context, filled, poly_x, poly_y, n, 1);
}

/* This function draws an ellipse with given mean and covariance parameters
   using the Cholesky decomposition. It is slightly faster and more
   elegant than the PCA method, but you can't draw the axes of the ellipse
   conveniently this way. */

void art_draw_ellipse(art_buffer_p buffer, art_context_p context, int filled,
		      float x, float y, float x_var, 
		      float xy_cov, float y_var, float k)
{
  float poly_x[MAX_POLY_POINTS], poly_y[MAX_POLY_POINTS], l11, l21, l22;
  int i;
  
  l11 = sqrt(x_var);
  l21 = xy_cov / l11;
  l22 = sqrt(y_var - carmen_square(l21));
  for(i = 0; i < MAX_POLY_POINTS; i++) {
    double t = i / (float)MAX_POLY_POINTS * 2 * M_PI, xt, yt;
    xt = cos(t);    yt = sin(t);
    poly_x[i] = x + l11 * xt * k;
    poly_y[i] = y + (l21 * xt + l22 * yt) * k;
  }
  art_draw_poly(buffer, context, filled, poly_x, poly_y, MAX_POLY_POINTS, 1);
}

/* This function draws an ellipse with a given mean and covariance parameters.
   It does this using PCA.  It figures out the eigenvectors and eigenvalues
   (major and minor axes) and draws the ellipse using this transformation
   of coordinates */

void art_draw_gaussian(art_buffer_p buffer, art_context_p context,
		       float x, float y, float x_var, 
		       float xy_cov, float y_var, float k)
{
  float poly_x[MAX_POLY_POINTS], poly_y[MAX_POLY_POINTS];
  double len, discriminant, eigval1, eigval2,
    eigvec1x, eigvec1y, eigvec2x, eigvec2y;
  int i, ex1, ey1, ex2, ey2;
  
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
    discriminant = sqrt(4 * carmen_square(xy_cov) + carmen_square(x_var - y_var));
    eigval1 = .5 * (x_var + y_var - discriminant);
    eigval2 = .5 * (x_var + y_var + discriminant);
    eigvec1x = (x_var - y_var - discriminant) / (2.0 * xy_cov);
    eigvec1y = 1.0;
    eigvec2x = (x_var - y_var + discriminant) / (2.0 * xy_cov);
    eigvec2y = 1.0;
    /* normalize eigenvectors */
    len = sqrt(carmen_square(eigvec1x) + 1.0);
    eigvec1x /= len;
    eigvec1y /= len;
    len = sqrt(carmen_square(eigvec2x) + 1.0);
    eigvec2x /= len;
    eigvec2y /= len;
  }

  /* take square root of eigenvalues and scale -- once this is
     done, eigvecs are unit vectors along axes and eigvals are
     corresponding radii */
  if (eigval1 < 0 || eigval2 < 0) {
    art_draw_circle(buffer, context, 0, x, y, 1);
    return;
  }
  eigval1 = sqrt(eigval1) * k;
  eigval2 = sqrt(eigval2) * k;
  if(eigval1 < 1)
    eigval1 = 1;
  if(eigval2 < 1)
    eigval2 = 1;
  
  /* compute points around edge of ellipse */
  for (i = 0; i < MAX_POLY_POINTS; i++) {
    double theta = M_PI * (-1.0 + 2.0 * i / (float)MAX_POLY_POINTS);
    double xi = cos(theta) * eigval1;
    double yi = sin(theta) * eigval2;
    poly_x[i] = xi * eigvec1x + yi * eigvec2x + x;
    poly_y[i] = xi * eigvec1y + yi * eigvec2y + y;
  }

  /* finally we can draw it */
  art_draw_poly(buffer, context, 0, poly_x, poly_y, MAX_POLY_POINTS, 1);
  
  ex1 = x + eigval1 * eigvec1x;
  ey1 = y + eigval1 * eigvec1y;
  ex2 = x - eigval1 * eigvec1x;
  ey2 = y - eigval1 * eigvec1y;
  art_draw_line(buffer, context, ex1, ey1, ex2, ey2);
  ex1 = x + eigval2 * eigvec2x;
  ey1 = y + eigval2 * eigvec2y;
  ex2 = x - eigval2 * eigvec2x;
  ey2 = y - eigval2 * eigvec2y;
  art_draw_line(buffer, context, ex1, ey1, ex2, ey2);
}

void art_draw_rectangle(art_buffer_p buffer, art_context_p context, int filled,
			float x_1, float y_1, float x_2, float y_2)
{
  float poly_x[4], poly_y[4];

  poly_x[0] = x_1;  poly_y[0] = y_1;
  poly_x[1] = x_2;  poly_y[1] = y_1;
  poly_x[2] = x_2;  poly_y[2] = y_2;
  poly_x[3] = x_1;  poly_y[3] = y_2;
  art_draw_poly(buffer, context, filled, poly_x, poly_y, 4, 1);
}

void art_draw_arc(art_buffer_p buffer, art_context_p context,
		  float x, float y, float r, float a1, float deltaa)
{
  int i, n = r / 20.0 * MAX_POLY_POINTS * deltaa / (2 * M_PI);
  float theta, poly_x[MAX_POLY_POINTS], poly_y[MAX_POLY_POINTS];

  if(n > MAX_POLY_POINTS)
    n = MAX_POLY_POINTS;
  else if(n < MIN_POLY_POINTS)
    n = MIN_POLY_POINTS;
  for(i = 0; i < n; i++) {
    theta = a1 + i / (float)(n - 1) * deltaa;
    poly_x[i] = x + r * cos(theta);
    poly_y[i] = y + r * sin(theta);
  }
  art_draw_poly(buffer, context, 0, poly_x, poly_y, n, 0);
}

void art_draw_wedge(art_buffer_p buffer, art_context_p context, int filled,
		    float x, float y, float r, float a1, float deltaa)
{
  float theta, poly_x[MAX_POLY_POINTS + 1], poly_y[MAX_POLY_POINTS + 1];
  int i;

  for(i = 0; i < MAX_POLY_POINTS; i++) {
    theta = a1 + i / (float)(MAX_POLY_POINTS - 1) * deltaa;
    poly_x[i] = x + r * cos(theta);
    poly_y[i] = y + r * sin(theta);
  }
  poly_x[MAX_POLY_POINTS] = x;
  poly_y[MAX_POLY_POINTS] = y;
  art_draw_poly(buffer, context, filled, 
		poly_x, poly_y, MAX_POLY_POINTS + 1, 1);
}

void art_draw_rgb_image(art_buffer_p dest_buffer, int dest_x, int dest_y,
			int dest_width, int dest_height, 
			unsigned char *src_buffer, int src_width,
			int src_height)
{
  double affine[6];

  affine[0] = dest_width / (float)src_width;
  affine[1] = 0;
  affine[2] = 0;
  affine[3] = dest_height / (float)src_height;
  affine[4] = dest_x;
  affine[5] = dest_buffer->height - dest_y - dest_height;

  art_rgb_affine(dest_buffer->buffer, 0, 0, dest_buffer->width,
		 dest_buffer->height, dest_buffer->width * 3,
		 (art_u8 *)src_buffer, src_width, src_height, 
		 src_width * 3, affine, ART_FILTER_NEAREST, NULL);
}

void art_draw_rgba_image(art_buffer_p dest_buffer, int dest_x, int dest_y,
			 int dest_width, int dest_height, 
			 unsigned char *src_buffer, int src_width,
			 int src_height)
{
  double affine[6];

  affine[0] = dest_width / (float)src_width;
  affine[1] = 0;
  affine[2] = 0;
  affine[3] = dest_height / (float)src_height;
  affine[4] = dest_x;
  affine[5] = dest_buffer->height - dest_y - dest_height;

  art_rgb_rgba_affine(dest_buffer->buffer, 0, 0, dest_buffer->width,
		      dest_buffer->height, dest_buffer->width * 3,
		      (art_u8 *)src_buffer, src_width, src_height, 
		      src_width * 3, affine, ART_FILTER_NEAREST, NULL);
}

void art_draw_transformed_rgb_image(art_buffer_p dest_buffer, 
				    unsigned char *src_buffer, int src_width,
				    int src_height, int dest_x, int dest_y,
				    float angle, float scale)
{
  double affine[6];

  affine[0] = scale * cos(angle);
  affine[1] = -scale * sin(angle);
  affine[2] = scale * sin(angle);
  affine[3] = scale * cos(angle);
  affine[4] = dest_x - src_height * scale * sin(angle);
  affine[5] = dest_buffer->height - dest_y - src_height * scale * cos(angle);

  art_rgb_affine(dest_buffer->buffer, 0, 0, dest_buffer->width,
		 dest_buffer->height, dest_buffer->width * 3,
		 (art_u8 *)src_buffer, src_width, src_height, 
		 src_width * 3, affine, ART_FILTER_NEAREST, NULL);
}

void art_draw_transformed_rgba_image(art_buffer_p dest_buffer, 
				     unsigned char *src_buffer, int src_width,
				     int src_height, int dest_x, int dest_y,
				     float angle, float scale)
{
  double affine[6];

  affine[0] = scale * cos(angle);
  affine[1] = -scale * sin(angle);
  affine[2] = scale * sin(angle);
  affine[3] = scale * cos(angle);
  affine[4] = dest_x - src_height * scale * sin(angle);
  affine[5] = dest_buffer->height - dest_y - src_height * scale * cos(angle);

  art_rgb_rgba_affine(dest_buffer->buffer, 0, 0, dest_buffer->width,
		      dest_buffer->height, dest_buffer->width * 3,
		      (art_u8 *)src_buffer, src_width, src_height, 
		      src_width * 4, affine, ART_FILTER_NEAREST, NULL);
}

int art_write_buffer_ppm(art_buffer_p buffer, char *filename)
{
  FILE *fp;
  
  fp = fopen(filename, "w");
  if(fp == NULL)
    return -1;
  fprintf(fp, "P6\n%d %d\n255\n", buffer->width, buffer->height);
  fwrite(buffer->buffer, buffer->height * buffer->width, 3, fp);
  fclose(fp);
  return 0;
}

int art_write_buffer_png(art_buffer_p buffer, char *filename)
{
  char tempfilename[100], cmd[100];
  
  strcpy(tempfilename, "/tmp/ppm_temp");
  if(art_write_buffer_ppm(buffer, tempfilename) < 0)
    return -1;
  sprintf(cmd, "pnmtopng %s > %s\n", tempfilename, filename);
  system(cmd);
  sprintf(cmd, "rm %s\n", tempfilename);
  fprintf(stderr, "Writing %s\n", filename);
  return 0;
}
