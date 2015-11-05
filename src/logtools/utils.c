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
#include <time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include <carmen/carmen.h>
#include <carmen/logtools.h>
#include "defines.h"

double
fsgn( double val )
{
  if (val>0.0) {
    return(1);
  } else if (val<0.0) {
    return(-1);
  } else {
    return(0);
  }
}

int
sgn( int val )
{
  if (val>0) {
    return(1);
  } else if (val<0) {
    return(-1);
  } else {
    return(0);
  }
}

double
rad2Deg(double x)
{
  return x * 57.29577951308232087679;
}

double
deg2Rad(double x)
{
  return x * 0.01745329251994329576;
}


double
normalize_theta(double theta) 
{
  int multiplier;

  if (theta >= -M_PI && theta < M_PI)
    return theta;

  multiplier = theta / (2*M_PI);
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
} 

int
timeCompare ( struct timeval time1, struct timeval time2 )
{
  if (time1.tv_sec<time2.tv_sec) {
    return(-1);
  } else if (time1.tv_sec>time2.tv_sec) {
    return(1);
  } else if (time1.tv_usec<time2.tv_usec) {
    return(-1);
  } else if (time1.tv_usec>time2.tv_usec) {
    return(1);
  } else {
    return(0);
  }
}

double
timeDiff( struct timeval  t1, struct timeval t2)
{
   double diff;
   diff =  (double) (t2.tv_usec - t1.tv_usec) / 1000000.0;
   diff += (double) (t2.tv_sec  - t1.tv_sec);
   return(diff);
}


logtools_vector2_t
logtools_rotate_vector2( logtools_vector2_t p, double rot )
{
  logtools_vector2_t v;
  v.x = p.x*cos(rot)-p.y*sin(rot);
  v.y = p.x*sin(rot)+p.y*cos(rot);
  return(v);
}

logtools_vector2_t
logtools_rotate_and_translate_vector2( logtools_vector2_t p, double rot,
			      logtools_vector2_t trans )
{
  logtools_vector2_t v;
  v.x = p.x*cos(rot)-p.y*sin(rot)+trans.x;
  v.y = p.x*sin(rot)+p.y*cos(rot)+trans.y;
  return(v);
}

double
logtools_vector2_distance( logtools_vector2_t p1, logtools_vector2_t p2 ) {
  return sqrt( (p1.x-p2.x)*(p1.x-p2.x) +
	       (p1.y-p2.y)*(p1.y-p2.y) );
}
  
double
logtools_vector2_length( logtools_vector2_t v1 )
{
  return( sqrt( (v1.x*v1.x) + (v1.y*v1.y) ) );
}


double
random_gauss( void )
{
  static int iset = 0;
  static double gset;
  float fac, rsq, v1, v2;
  if(iset == 0) {
    do {
      v1 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;
      v2 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;         
      rsq = v1*v1 + v2*v2;
    } while(rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0*log(rsq)/rsq);
    gset = v1*fac;
    iset = 1;
    return v2*fac;
  }
  else {
    iset = 0;
    return gset;
  }
}

double
logtools_gauss_function( double x, double mu, double sigma )
{
 return( (1/sqrt(2.0*M_PI*sigma*sigma)) *
	 exp(-(((x-mu)*(x-mu))/(2*sigma*sigma))) );  
}

logtools_gauss_kernel_t
logtools_compute_gauss_kernel_with_function( int length )
{
  logtools_gauss_kernel_t kernel;
  
  int center = (length-1) / 2;
  double sigma = sqrt( length/(2*M_PI) );
  int i;

  kernel.len = length;
  kernel.val = (double *) malloc( length * sizeof(double) );
  for (i=0;i<length;i++) {
    kernel.val[i] = logtools_gauss_function( (i-center), 0.0, sigma );
  }
  return(kernel);
}

logtools_gauss_kernel_t
logtools_compute_gauss_kernel(int length )
{
  logtools_gauss_kernel_t  kernel;
  int i, j, *store, sum = 0;
  store = (int *) malloc( length*sizeof(int) );
  store[0] = 1;
  for ( i=0; i<length-1; i++ ) {
    store[i+1] = 1;
    for ( j=i; j>0; j-- ) {
      store[j] = store[j] + store[j-1];
    }
  }
  for ( i=0; i<length; i++ ) {
    sum += store[i];
  }
  kernel.len = length;
  kernel.val = (double *) malloc( length * sizeof(double) );
  for (i=0;i<length;i++) {
    kernel.val[i] = store[i] / (double) sum;
  }
  free( store );
  return(kernel);
}


double
convert_orientation_to_range( double angle ) {
  if (angle<-M_PI) {
    return(2*M_PI -  fmod( fabs(angle),2*M_PI ));
  } else if (angle>M_PI) {
    return(-2*M_PI + fmod( angle,2*M_PI ));
  } else {
    return(angle);
  }
}

double
compute_orientation_diff( double start, double end ) {
  double diff;
  diff =
    convert_orientation_to_range( start ) -
    convert_orientation_to_range( end );
  if (diff<-M_PI) {
    return(2*M_PI+diff);
  } else if (diff>M_PI) {
    return(-2*M_PI+diff);
  } else {
    return(diff);
  }
}

logtools_rmove2_t
logtools_movement2_between_rpos2( logtools_rpos2_t start, logtools_rpos2_t end )
{
  logtools_rmove2_t move;

  /* compute forward and sideward sensing_MOVEMENT */
  move.forward =
    + (end.y - start.y) * sin(start.o)
    + (end.x - start.x) * cos(start.o);
  
  move.sideward =
    - (end.y - start.y) * cos(start.o)
    + (end.x - start.x) * sin(start.o);
  
  move.rotation = compute_orientation_diff( end.o, start.o );

  return( move );
}

logtools_rpos2_t
logtools_rpos2_with_movement2( logtools_rpos2_t start, logtools_rmove2_t move ) {
  logtools_rpos2_t end;
  if ( (move.forward==0.0) && (move.sideward==0.0) && (move.rotation==0.0) )
    return (start);
  end.x = start.x + cos(start.o) * move.forward + sin(start.o) * move.sideward;
  end.y = start.y + sin(start.o) * move.forward - cos(start.o) * move.sideward;
  end.o = convert_orientation_to_range( start.o + move.rotation );
  return(end);
}

logtools_rpos2_t
logtools_rpos2_backwards_with_movement2( logtools_rpos2_t start, logtools_rmove2_t move ) {
  logtools_rpos2_t end;
  if ( (move.forward==0.0) && (move.sideward==0.0) && (move.rotation==0.0) )
    return (start);
  end.o = convert_orientation_to_range( start.o - move.rotation );
  end.x = start.x - cos(end.o) * move.forward - sin(end.o) * move.sideward;
  end.y = start.y - sin(end.o) * move.forward + cos(end.o) * move.sideward;
  return(end);
}

double
get_user_time( void )
{
  struct tms     tms_time;
  if ((int)times(&tms_time)==-1) {
    return(-1);
  }
  return( (double)tms_time.tms_utime / (double)CLOCKS_PER_SEC );
}
  
void
convert_time( double tval, struct timeval *time )
{
  time->tv_sec = (long) floor(tval);
  time->tv_usec = (long) ( (tval-(double) time->tv_sec) * 1000000 );
}

double
double_time( struct timeval time )
{
  return( (double) (time.tv_sec) + (double) (time.tv_usec/1000000.0) );
}

double
sick_laser_fov( int num )
{
  switch( num ) {
  case(180):
    /* 180 degrees */
    return(M_PI);
    break;
  case(181):
    return(M_PI);
    break;
  case(360):
    return(M_PI);
    break;
  case(361):
    return(M_PI);
    break;
  case(401):
    /* 100 degrees */
    return(1.74532925199432957692);
    break;
  case(721):
    return(M_PI);
    break;
  default:
    return(M_PI);
    break;
  }
}

double
sick_laser_angle_diff( int num, double fov )
{
  switch( num ) {
  case(180):
    return(fov / (double) num);
    break;
  case(360):
    return(fov / (double) num);
    break;
  case(400):
    return(fov / (double) num);
    break;
  default:
    return(fov / (double) (num-1));
    break;
  }
}

double
logtools_rpos2_distance( logtools_rpos2_t p1, logtools_rpos2_t p2 )
{
  return sqrt( (p1.x-p2.x)*(p1.x-p2.x) +
	       (p1.y-p2.y)*(p1.y-p2.y) );
}
  
char *
extended_filename( char * filename )
{
  struct timeval     time;
  struct tm        * date;
  struct utsname     info;

  static char        fname[MAX_NAME_LENGTH];
  static char        addstr[MAX_NAME_LENGTH];

  int                i, j, ctr;

  ctr = 0;
  for (i=0;i<(int)strlen(filename);i++) {
    if (filename[i]=='%') {
      switch (filename[i+1]) {
      case 'd':	
	gettimeofday( &time, NULL );
	date = localtime( &(time.tv_sec) );
	snprintf( addstr, MAX_NAME_LENGTH, "%d-%s%d-%s%d",
		  1900+date->tm_year,
		  (date->tm_mon<10)?"0":"",
		  date->tm_mon,
		  (date->tm_mday<10)?"0":"",
		  date->tm_mday );
	
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 't':	
	gettimeofday( &time, NULL );
	date = localtime( &(time.tv_sec) );
	snprintf( addstr, MAX_NAME_LENGTH, "%s%d:%s%d",
		  (date->tm_hour<10)?"0":"",
		  date->tm_hour,
		  (date->tm_min<10)?"0":"",
		  date->tm_min );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'h':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.nodename);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.nodename[j];
	  }
	}
	i++;
	break;
      case 'n':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.nodename);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.nodename[j];
	  }
	}
	i++;
	break;
      case 's':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.sysname);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.sysname[j];
	  }
	}
	i++;
	break;
      case 'r':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.release);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.release[j];
	  }
	}
	i++;
	break;
      case 'm':	
	if (!uname( &info )) {
	  for (j=0;j<(int)strlen(info.machine);j++) {
	    if (ctr<MAX_NAME_LENGTH)
	      fname[ctr++]=info.machine[j];
	  }
	}
	i++;
	break;
      default:
	if (ctr<MAX_NAME_LENGTH)
	  fname[ctr++]=filename[i];
	break;
      }
    } else {
      if (ctr<MAX_NAME_LENGTH)
	fname[ctr++]=filename[i];
    }
  }
  if (ctr<MAX_NAME_LENGTH) {
    fname[ctr++]='\0';
  } else {
    fname[MAX_NAME_LENGTH-1]='\0';
  }
  return(fname);
}

char *
printable_filename( char * filename )
{
  static char        fname[MAX_NAME_LENGTH];
  static char        addstr[MAX_NAME_LENGTH];

  int                i, j, ctr;

  ctr = 0;
  for (i=0;i<(int)strlen(filename);i++) {
    if (filename[i]=='%') {
      switch (filename[i+1]) {
      case 'd':	
	snprintf( addstr, MAX_NAME_LENGTH, "<DATE>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 't':	
	snprintf( addstr, MAX_NAME_LENGTH, "<TIME>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'h':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<HOST>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'n':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<NODENAME>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 's':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<SYSNAME>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'r':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<RELEASE>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      case 'm':	
 	snprintf( addstr, MAX_NAME_LENGTH, "<MACHINE>" );
	for (j=0;j<(int)strlen(addstr);j++) {
	  if (ctr<MAX_NAME_LENGTH)
	    fname[ctr++]=addstr[j];
	}
	i++;
	break;
      default:
	if (ctr<MAX_NAME_LENGTH)
	  fname[ctr++]=filename[i];
	break;
      }
    } else {
      if (ctr<MAX_NAME_LENGTH)
	fname[ctr++]=filename[i];
    }
  }
  if (ctr<MAX_NAME_LENGTH) {
    fname[ctr++]='\0';
  } else {
    fname[MAX_NAME_LENGTH-1]='\0';
  }
  return(fname);
}

FILE *
fopen_filename( char * filename, char * mode )
{
  char fname[MAX_NAME_LENGTH];
  strncpy( fname, extended_filename( filename ), MAX_NAME_LENGTH );
  return(fopen( fname, mode));
}
    
  
double
gps_degree_abs_decimal_100( double degree_minute )
{
  double gps_deg, gps_min, gps_sec;
  gps_deg = floor( degree_minute/100.0 );
  gps_min = floor( degree_minute )-gps_deg*100;
  gps_sec = degree_minute-floor(degree_minute);
  return(gps_deg + (gps_min / 60.0) + (gps_sec / 60.0)); 
}

double
gps_degree_abs_decimal( double degree_minute )
{
  double gps_deg, gps_min, gps_sec;
  gps_deg = floor( degree_minute );
  gps_min = floor( degree_minute )-gps_deg;
  gps_sec = degree_minute-floor(degree_minute);
  return(gps_deg + (gps_min / 60.0) + (gps_sec / 60.0)); 
}

double
gps_degree_decimal( double degree_minute, char orient )
{
  if ( (orient=='w') || (orient=='s') ||
       (orient=='W') || (orient=='S') ) {
    return(-gps_degree_abs_decimal(degree_minute));
  } else {
    return(gps_degree_abs_decimal(degree_minute));
  }
}

logtools_vector2_t
logtools_compute_laser_points( logtools_rpos2_t rpos, double val,
			       logtools_rmove2_t offset, double angle )
{
  logtools_vector2_t abspt;
  abspt.x =
    rpos.x + 
    cos( angle+offset.rotation+rpos.o ) * val;
  abspt.y =
    rpos.y + 
    sin( angle+offset.rotation+rpos.o ) * val;
  return(abspt);
}



logtools_laser_coord2_t
compute_laser2d_coord( logtools_lasersens2_data_t lsens, int i )
{
  double                   val;
  logtools_laser_coord2_t  coord = { {0.0,0.0},{0.0,0.0}, 0, 0 };
  logtools_rpos2_t         rpos, npos;

  rpos  = logtools_rpos2_with_movement2( lsens.estpos,
					lsens.laser.offset );
  npos.x = 0.0;
  npos.y = 0.0;
  npos.o = 0.0;

  val = lsens.laser.val[i];
  coord.relpt = logtools_compute_laser_points( npos, val,
					       lsens.laser.offset,
					       lsens.laser.angle[i] );
  coord.abspt = logtools_compute_laser_points( rpos, val,
					       lsens.laser.offset,
					       lsens.laser.angle[i] );
  return(coord);
}

void
logtools_compute_coordpts( logtools_log_data_t *rec )
{
  int i, j;
  for (j=0;j<rec->numlaserscans;j++) {
    if (rec->lsens[j].coord==NULL) {
      rec->lsens[j].coord =
	(logtools_laser_coord2_t *) malloc( rec->lsens[j].laser.numvalues *
				 sizeof(logtools_laser_coord2_t) );
      for (i=0;i<rec->lsens[j].laser.numvalues;i++) {
	rec->lsens[j].coord[i] = compute_laser2d_coord(rec->lsens[j], i);
      }
    }
  }
}

