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

typedef struct {
  double    y;
  double    u;
  double    v;
} YUV;

typedef struct {
  double    r;
  double    g;
  double    b;
} RGB;

typedef struct {
  double    h;
  double    s;
  double    v;
} HSV;


YUV           convert_from_rgb( RGB color );

RGB           convert_from_yuv( YUV color );

RGB           hsv_to_rgb( HSV color );

RGB           val_to_rgb( double val );

RGB           val_to_gray( double val );



typedef void (*logtools_draw_function_t)(int x,int y);

void          logtools_draw_set_function( logtools_draw_function_t funct );

void          logtools_draw_ellipse( logtools_ivector2_t p, const int a,const int b);

void          logtools_draw_line( logtools_ivector2_t p1, logtools_ivector2_t p2 );
     
void          logtools_draw_circle( logtools_ivector2_t p, const int r );

void          logtools_draw_filled_circle( logtools_ivector2_t p, const int r );

