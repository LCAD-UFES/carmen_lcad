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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include <carmen/carmen.h>
#include <carmen/logtools.h>
#include <carmen/logtools_graphics.h>

logtools_draw_function_t d_fct = NULL;

void
logtools_putpixel( int x, int y )
{
  if (d_fct!=NULL)
    d_fct( x, y );
}

void
logtools_draw_set_function( logtools_draw_function_t funct )
{
  d_fct = funct;
}

void
logtools_draw_ellipse( logtools_ivector2_t pt, const int a, const int b)
{
  float aa=(a*a);
  float bb=(b*b);
  float aa2=(aa*2);
  float bb2=(bb*2);
  
  float x=0;
  float y=b;
  
  float fx=0;
  float fy=(aa2*b);
  
  float p = (int) (bb-(aa*b)+(0.25*aa)+0.5);
  
  logtools_putpixel( (pt.x+x), (pt.y+y) );
  logtools_putpixel( (pt.x+x), (pt.y-y) );
  logtools_putpixel( (pt.x-x), (pt.y-y) );
  logtools_putpixel( (pt.x-x), (pt.y+y) );
  
  while(fx<fy) {
    x++;
    fx+=bb2;
    
    if(p<0) {
      p+=(fx+bb);
    } else {
      y--;
      fy-=aa2;
      p+=(fx+bb-fy);
    }
    
    logtools_putpixel( (pt.x+x), (pt.y+y) );
    logtools_putpixel( (pt.x+x), (pt.y-y) );
    logtools_putpixel( (pt.x-x), (pt.y-y) );
    logtools_putpixel( (pt.x-x), (pt.y+y) );
  }
  
  p = (int) ((bb*(x+0.5)*(x+0.5))+(aa*(y-1)*(y-1))-(aa*bb)+0.5);
  
  while(y>0) {
    y--;
    fy-=aa2;
    
    if(p>=0) {
      p+=(aa-fy);
    } else {
      x++;
      fx+=bb2;
      p+=(fx+aa-fy);
    }
    
    logtools_putpixel( (pt.x+x), (pt.y+y) );
    logtools_putpixel( (pt.x+x), (pt.y-y) );
    logtools_putpixel( (pt.x-x), (pt.y-y) );
    logtools_putpixel( (pt.x-x), (pt.y+y) );
  }
}

void
logtools_draw_line( logtools_ivector2_t p1, logtools_ivector2_t p2 )
{
  int p, x, y, x1, y1, x2, y2, dx, dy;
  int two_dx, two_dx_dy, two_dy, two_dy_dx, inc_dec;
  
  x1=p1.x;  y1=p1.y;
  x2=p2.x;  y2=p2.y;

  if (p1.x>p2.x) {
    x1=p2.x;  y1=p2.y;
    x2=p1.x;  y2=p1.y;
  }

  dx=abs(x2-x1);
  dy=abs(y2-y1);
  inc_dec=((y2>=y1)?1:-1);

  if(dx>dy) {
    two_dy=(2*dy);
    two_dy_dx=(2*(dy-dx));
    p=((2*dy)-dx);
    
    x=x1;
    y=y1;
    
    logtools_putpixel( x, y );

    while(x<x2) {
      x++;
      
      if(p<0) {
	p+=two_dy;
      } else {
	y+=inc_dec;
	p+=two_dy_dx;
      }
      
      logtools_putpixel( x, y );
    }
    
  } else {
    
    two_dx=(2*dx);
    two_dx_dy=(2*(dx-dy));
    p=((2*dx)-dy);
    
    x=x1;
    y=y1;
    
    logtools_putpixel( x, y );
    
    while(y!=y2) {
      y+=inc_dec;
      
      if(p<0) {
	p+=two_dx;
      } else {
	x++;
	p+=two_dx_dy;
      }
      
      logtools_putpixel( x, y );
    }
  }
}

void
logtools_draw_circle( logtools_ivector2_t p, const int r )
{
  int x=0;
  int y=r;
  int n=(1-r);

  do {
    
    logtools_putpixel( (p.x+x), (p.y+y) );
    logtools_putpixel( (p.x+y), (p.y+x) );
    logtools_putpixel( (p.x+y), (p.y-x) );
    logtools_putpixel( (p.x+x), (p.y-y) );
    logtools_putpixel( (p.x-x), (p.y-y) );
    logtools_putpixel( (p.x-y), (p.y-x) );
    logtools_putpixel( (p.x-y), (p.y+x) );
    logtools_putpixel( (p.x-x), (p.y+y) );
    
    x++;
    
    if(n<0) {
      n+=((2*x)+1);
    } else {
      y--;
      n+=((2*(x-y))+1);
    }
  } while(x<=y);
}

void
logtools_draw_filled_circle( logtools_ivector2_t p, const int r )
{
  logtools_ivector2_t p1, p2;
  int count, counter = (p.y+r);

  for ( count=(p.y-r); count<=counter; count++ ) {
    p1.x = (int) (p.x+sqrt((r*r)-((count-p.y)*(count-p.y)))+0.5);
    p1.y = count;
    p2.x = (int) (p.x-sqrt((r*r)-((count-p.y)*(count-p.y)))+0.5);
    p2.y = count;
    logtools_draw_line( p1, p2 );
  }
  
}
