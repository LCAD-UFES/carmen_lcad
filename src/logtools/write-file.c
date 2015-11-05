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
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <fnmatch.h>

#include <carmen/global.h>
#include <carmen/logtools.h>

#include "defines.h"
#include "internal.h"

#define MAX_NAME_LENGTH    256


void
write_script_time( FILE *fpout, unsigned long sec, unsigned long usec )
{
  struct tm   * actual_date;
  time_t        tsec;
  double        fsec;
  
  tsec = sec;
  actual_date = localtime( &tsec );
  fsec =  (double) (actual_date->tm_sec+(usec/1000000.0));
  fprintf( fpout, "@SENS %s%d-%s%d-%d %s%d:%s%d:%s%f\n",
	   actual_date->tm_mday<10?"0":"",
	   actual_date->tm_mday,
	   actual_date->tm_mon<10?"0":"",
	   actual_date->tm_mon,
	   actual_date->tm_year>99?
	   actual_date->tm_year-100:actual_date->tm_year,
	   actual_date->tm_hour<10?"0":"",
	   actual_date->tm_hour,
	   actual_date->tm_min<10?"0":"",
	   actual_date->tm_min,
	   fsec<10?"0":"",
	   fsec );
}

void
write_script_open( FILE *fpout, unsigned long sec, unsigned long usec )
{
  struct tm   * actual_date;
  time_t        tsec;
  double        fsec;
  
  tsec = sec;
  actual_date = localtime( &tsec );
  fsec =  (double) (actual_date->tm_sec+(usec/1000000.0));
  fprintf( fpout, "@OPEN %s%d-%s%d-%d %s%d:%s%d:%s%f\n",
	   actual_date->tm_mday<10?"0":"",
	   actual_date->tm_mday,
	   actual_date->tm_mon<10?"0":"",
	   actual_date->tm_mon,
	   actual_date->tm_year>99?
	   actual_date->tm_year-100:actual_date->tm_year,
	   actual_date->tm_hour<10?"0":"",
	   actual_date->tm_hour,
	   actual_date->tm_min<10?"0":"",
	   actual_date->tm_min,
	   fsec<10?"0":"",
	   fsec );
}

int
write_script_file( char *filename, logtools_log_data_t *rec )
{
  FILE  * iop;
  int     i, k, stop;

  fprintf( stderr, "# write script file %s ...\n", filename );
  if ((iop = fopen( filename, "w")) == 0){
    fprintf(stderr, "# WARNING: can't write script file %s\n", filename );
    return(-1);
  }
  
  stop = FALSE;
  for (i=0; i<rec->numentries && !stop; i++) {
    switch( rec->entry[i].type ) {
    case LASER_VALUES:
      write_script_open( iop,
			 rec->lsens[rec->entry[i].index].laser.time.tv_sec,
			 rec->lsens[rec->entry[i].index].laser.time.tv_usec );
      stop = TRUE;
      break;
    case POSITION:
      write_script_open( iop,
			 rec->psens[rec->entry[i].index].time.tv_sec,
			 rec->psens[rec->entry[i].index].time.tv_usec );
      stop = TRUE;
      break;
    default:
      break;
    }
  }
  

  for (i=0; i<rec->numentries; i++) {
    switch( rec->entry[i].type ) {
    case LASER_VALUES:
      write_script_time( iop,
			 rec->lsens[rec->entry[i].index].laser.time.tv_sec,
			 rec->lsens[rec->entry[i].index].laser.time.tv_usec );
      fprintf( iop, "#LASER %d 0:", rec->lsens[rec->entry[i].index].laser.numvalues );
      for (k=0; k<rec->lsens[rec->entry[i].index].laser.numvalues; k++) {
	fprintf( iop, " %d", (int) rec->lsens[rec->entry[i].index].laser.val[k] );
      }
      fprintf( iop, "\n" );
      break;
    case POSITION:
      write_script_time( iop,
			 rec->psens[rec->entry[i].index].time.tv_sec,
			 rec->psens[rec->entry[i].index].time.tv_usec );
      fprintf( iop, "#ROBOT %f %f %f\n",
	       (float) rec->psens[rec->entry[i].index].rpos.x,
	       (float) rec->psens[rec->entry[i].index].rpos.y,
	       (float) (90.0-rad2deg(rec->psens[rec->entry[i].index].rpos.o)) );
      
      break;
    default:
      break;
    }
  }
    
  return(0);
}

int
write_rec2d_file( char *filename, logtools_log_data_t *rec )
{
  FILE  * iop;
  int     i, k;
  
  fprintf( stderr, "# write rec file %s ...\n", filename );
  if ((iop = fopen( filename, "w")) == 0){
    fprintf(stderr, "# WARNING: can't write rec file %s\n", filename );
    return(-1);
  }

  for (i=0; i<rec->numentries; i++) {
    switch( rec->entry[i].type ) {
    case LASER_VALUES:
      fprintf(iop, "LASER-RANGE %11ld %7ld %d %d %.1f:",
	      (long int)rec->lsens[rec->entry[i].index].laser.time.tv_sec,
	      (long int)rec->lsens[rec->entry[i].index].laser.time.tv_usec,
	      rec->lsens[rec->entry[i].index].id,
	      rec->lsens[rec->entry[i].index].laser.numvalues,
	      rad2deg(rec->lsens[rec->entry[i].index].laser.fov) );
      for (k=0;k<rec->lsens[rec->entry[i].index].laser.numvalues;k++) {
	fprintf(iop, " %.1f", rec->lsens[rec->entry[i].index].laser.val[k] );
      }
      fprintf(iop, "\n" );
      break;
    case POSITION:
      fprintf( iop, "POS %11ld %7ld: %.5f %.5f %.5f %.5f %.5f\n",
	       (long int)rec->psens[rec->entry[i].index].time.tv_sec,
	       (long int)rec->psens[rec->entry[i].index].time.tv_usec,
	       rec->psens[rec->entry[i].index].rpos.x,
	       rec->psens[rec->entry[i].index].rpos.y,
	       rad2deg(rec->psens[rec->entry[i].index].rpos.o),
	       rec->psens[rec->entry[i].index].rvel.tv,
	       rec->psens[rec->entry[i].index].rvel.rv );
      break;
    case MARKER:
      fprintf( iop, "MARKER %11ld %7ld: %s\n",
	       (long int)rec->marker[rec->entry[i].index].time.tv_sec,
	       (long int)rec->marker[rec->entry[i].index].time.tv_usec,
	       rec->marker[rec->entry[i].index].datastr );
      break;
    case UNKNOWN:
      break;
    default:
      break;
    }
  }
  return(0);
}

int
write_carmen_file( char *filename, logtools_log_data_t *rec )
{
  double    time, starttime = 0.0;
  int       i, k, stop; 
  FILE    * iop;
 
  fprintf( stderr, "# write carmen file %s ...\n", filename );
  if ((iop = fopen( filename, "w")) == 0){
    fprintf(stderr, "# WARNING: can't write carmen file %s\n", filename );
    return(-1);
  }
  
  stop = FALSE;
  for (i=0; i<rec->numentries && !stop; i++) {
    switch( rec->entry[i].type ) {
    case LASER_VALUES:
      starttime = double_time(rec->lsens[rec->entry[i].index].laser.time);
      stop = TRUE;
      break;
    case POSITION:
      starttime = double_time(rec->psens[rec->entry[i].index].time);
      stop = TRUE;
      break;
    default:
      break;
    }
  }
  
  fprintf( iop, "# message_name [message contents] ipc_timestamp ipc_hostname logger_timestamp\n" );
  fprintf( iop, "# message formats defined: PARAM SYNC ODOM FLASER RLASER TRUEPOS \n" );
  fprintf( iop, "# PARAM param_name param_value\n" );
  fprintf( iop, "# SYNC tagname\n" );
  fprintf( iop, "# ODOM x y theta tv rv accel\n" );
  fprintf( iop, "# FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta\n" );
  fprintf( iop, "# RLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta\n" );
  fprintf( iop, "# TRUEPOS true_x true_y true_theta odom_x odom_y odom_theta\n");
  fprintf( iop, "# NMEA-GGA utc latitude lat_orient longitude long_orient gps_quality num_sattelites hdop sea_level alitude geo_sea_level geo_sep data_age\n");
  
  for (i=0; i<rec->numentries; i++) {
    switch( rec->entry[i].type ) {
    case LASER_VALUES:
      time = double_time( rec->lsens[rec->entry[i].index].laser.time);
      switch( rec->lsens[rec->entry[i].index].id ) {
      case 0:
	fprintf( iop, "FLASER" );
	break;
      case 1:
	fprintf( iop, "RLASER" );
	break;
      case 2:
	fprintf( iop, "LASER2" );
	break;
      case 3:
	fprintf( iop, "LASER3" );
	break;
      default:
	fprintf( iop, "FLASER" );
	break;
      }
      fprintf( iop, " %d", rec->lsens[rec->entry[i].index].laser.numvalues );
      for (k=0;k<rec->lsens[rec->entry[i].index].laser.numvalues;k++) {
	fprintf(iop, " %.3f", rec->lsens[rec->entry[i].index].laser.val[k]/100.0 );
      }
      fprintf( iop, " %f %f %f %f %f %f %f %s %f\n",
	       (float) (rec->lsens[rec->entry[i].index].estpos.x/100.0),
	       (float) (rec->lsens[rec->entry[i].index].estpos.y/100.0),
	       (float) (carmen_normalize_theta(rec->lsens[rec->entry[i].index].estpos.o)),
	       (float) (rec->lsens[rec->entry[i].index].estpos.x/100.0),
	       (float) (rec->lsens[rec->entry[i].index].estpos.y/100.0),
	       (float) (carmen_normalize_theta(rec->lsens[rec->entry[i].index].estpos.o)),
	       time, "nohost", time-starttime );

      break;
    case POSITION:
      time = double_time(rec->psens[rec->entry[i].index].time);
      fprintf( iop, "ODOM %f %f %f %f %f %f %f %s %f\n",
	       (float) (rec->psens[rec->entry[i].index].rpos.x/100.0),
	       (float) (rec->psens[rec->entry[i].index].rpos.y/100.0),
	       (float) (carmen_normalize_theta(rec->psens[rec->entry[i].index].rpos.o)),
	       (float) (rec->psens[rec->entry[i].index].rvel.tv/100.0),
	       (float) (rec->psens[rec->entry[i].index].rvel.rv/100.0),
	       0.0, /* acceleration */
	       time, "nohost", time-starttime );
      break;
    default:
      break;
    }
  }
  return(0);
}

int
write_moos_file( char *filename, logtools_log_data_t *rec )
{
  FILE          * iop;
  struct tm     * actual_date; 
  int             i, k, stop;
  double          starttime = 0, time;
  long            ticks;

  fprintf( stderr, "# write moos file %s ...\n", filename );
  if ((iop = fopen( extended_filename(filename), "w")) == 0){
    fprintf(stderr, "# WARNING: can't write moos file %s\n", filename );
    return(-1);
  }

  stop = FALSE;
  for (i=0; i<rec->numentries && !stop; i++) {
    switch( rec->entry[i].type ) {
    case LASER_VALUES:
      starttime = double_time(rec->lsens[rec->entry[i].index].laser.time);
      stop = TRUE;
      break;
    case POSITION:
      starttime = double_time(rec->psens[rec->entry[i].index].time);
      stop = TRUE;
      break;
    default:
      break;
    }
  }
  
  fprintf( iop, "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n" );
  fprintf( iop, "%%%% LOG FILE:       %s\n", filename );
  if (starttime>0) {
    ticks = (long) starttime;
    actual_date = localtime( &ticks );
    fprintf( iop, "%%%% FILE WRITTEN:   %s%d.%s%d.%d %s%d:%s%d %s\n", 
	     (actual_date->tm_mday<10)?"0":"",
	     actual_date->tm_mday,
	     (actual_date->tm_mon<10)?"0":"",
	     actual_date->tm_mon,
	     1900+actual_date->tm_year,
	     (actual_date->tm_hour<10)?"0":"",
	     actual_date->tm_hour,
	     (actual_date->tm_min<10)?"0":"",
	     actual_date->tm_min,
	     actual_date->tm_isdst?"(DST)":"" );
    fprintf( iop, "%%%% LOGSTART        %.8f\n", starttime );
  }
  fprintf( iop, "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n" );

  for (i=0; i<rec->numentries; i++) {
    switch( rec->entry[i].type ) {
    case LASER_VALUES:
      time = double_time( rec->lsens[rec->entry[i].index].laser.time );
      fprintf( iop, "%.3f \tLASER_RANGE \t\t%s \ttime=%.6f,range[%d]{",
	       time-starttime,
	       "iLMS",
	       time,
	       rec->lsens[rec->entry[i].index].laser.numvalues );
      for (k=0;k<rec->lsens[rec->entry[i].index].laser.numvalues;k++) {
	fprintf(iop, "  %.3f%s",
		rec->lsens[rec->entry[i].index].laser.val[k]/100.0,
		k<rec->lsens[rec->entry[i].index].laser.numvalues-1?",":" " );
      }
      fprintf(iop, "}\n" );
      break;
    case POSITION:
      time = double_time(rec->psens[rec->entry[i].index].time);
      fprintf( iop, "%.3f \tROBOT_POSITION \t\t%s \t"
	       "timestamp=%.6f,x=%.6f,y=%.6f,theta=%.6f\n",
	       time-starttime,
	       "iRobot",
	       time,
	       (float) (rec->psens[rec->entry[i].index].rpos.x/100.0),
	       (float) (rec->psens[rec->entry[i].index].rpos.y/100.0),
	       (float) (rad2deg(rec->psens[rec->entry[i].index].rpos.o)-90.0) );
      break;
    default:
      break;
    }
  }
  return(0);
}

int
write_player_file( char *filename __attribute__ ((unused)),
		   logtools_log_data_t *rec __attribute__ ((unused)) )
{
  fprintf( stderr,
	   "# ERROR: writing for player-format currently not supported!\n" );
  return(1);
}

int
write_saphira_file( char *filename, logtools_log_data_t *rec, double dist )
{ 
  FILE    * iop;
  int       i, k, fov, cnt = 1;
  logtools_rpos2_t     lastpos;
  
  fprintf( stderr, "# write saphira file %s ...\n", filename );
  if ((iop = fopen( filename, "w")) == 0){
    fprintf(stderr, "# WARNING: can't write saphira file %s\n", filename );
    return(-1);
  }
  
  fprintf(iop, "LaserOdometryLog\n" );
  fprintf(iop, "version: 1\n" );
  if (rec->numlaserscans>0) {
    fprintf(iop, "sick1pose: 0 0 0\n" );
    fov = (int) (rad2deg(rec->lsens[0].laser.fov)+0.5);
    fprintf(iop, "sick1conf: %d %d %d\n",
	    -(fov/2), (fov/2), rec->lsens[0].laser.numvalues );
  }
  for (i=0; i<rec->numentries; i++) {
    switch( rec->entry[i].type ) {
    case LASER_VALUES:
      fprintf( iop, "#scan%d\n", cnt++ );
      if (cnt>2 &&
	  logtools_rpos2_distance( lastpos,
			    rec->lsens[rec->entry[i].index].estpos ) > dist ) {
	fprintf( iop, "robot: %.5f %.5f %.5f\n",
		 rec->lsens[rec->entry[i].index].estpos.x*10.0,
		 rec->lsens[rec->entry[i].index].estpos.y*10.0,
		 rad2deg(rec->lsens[rec->entry[i].index].estpos.o) );
	fprintf( iop, "robot_id: %d\n", 0 );
	fprintf( iop, "sick1:" );
	for (k=0;k<rec->lsens[rec->entry[i].index].laser.numvalues;k++) {
	  fprintf(iop, " %d",
		  (int) (rec->lsens[rec->entry[i].index].laser.val[k]*10.0) );
	}
	fprintf(iop, "\n" );
	lastpos = rec->lsens[rec->entry[i].index].estpos;
      }
      break;
    default:
      break;
    }
  }
  return(0);
}
  
int
logtools_write_logfile( logtools_log_data_t *rec, char * filename )
{
  enum logtools_file_t     out_type = REC;
  char                     fname[MAX_NAME_LENGTH];

  if ( !fnmatch( "script:*", filename, 0) ) {
    fprintf( stderr, "# INFO: write script-file-type!\n" );
    strncpy( fname, &(filename[7]), MAX_NAME_LENGTH );
    out_type = SCRIPT;
  } else if ( !fnmatch( "rec:*", filename, 0) ) {
    fprintf( stderr, "# INFO: write rec-file-type!\n" );
    strncpy( fname, &(filename[4]), MAX_NAME_LENGTH );
    out_type = REC;
  } else if ( !fnmatch( "carmen:*", filename, 0) ) {
    fprintf( stderr, "# INFO: write carmen-file-type!\n" );
    strncpy( fname, &(filename[7]), MAX_NAME_LENGTH );
    out_type = CARMEN;
  } else if ( !fnmatch( "moos:*", filename, 0) ) {
    fprintf( stderr, "# INFO: write moos-file-type!\n" );
    strncpy( fname, &(filename[5]), MAX_NAME_LENGTH );
    out_type = MOOS;
  } else if ( !fnmatch( "player:*", filename, 0) ) {
    fprintf( stderr, "# INFO: write player-file-type!\n" );
    strncpy( fname, &(filename[7]), MAX_NAME_LENGTH );
    out_type = PLAYER;
  } else if ( !fnmatch( "saphira:*", filename, 0) ) {
    fprintf( stderr, "# INFO: write player-file-type!\n" );
    strncpy( fname, &(filename[8]), MAX_NAME_LENGTH );
    out_type = SAPHIRA;
  } else if ( !fnmatch( "*" FILE_SCRIPT_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: use script-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    out_type = SCRIPT;
  } else if ( !fnmatch( "*" FILE_REC_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: write rec-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    out_type = REC;
  } else if ( !fnmatch( "*"FILE_CARMEN_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: write carmen-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    out_type = CARMEN;
  } else if ( !fnmatch( "*" FILE_MOOS_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: write moos-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    out_type = MOOS;
  } else if ( !fnmatch( "*" FILE_PLAYER_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: write player-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    out_type = PLAYER;
  } else if ( !fnmatch( "*" FILE_SAPHIRA_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: write saphira-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    out_type = SAPHIRA;
  }

  switch (out_type) {
  case SCRIPT:
    if (write_script_file( fname, rec ) !=0 )
      return(FALSE);
    break;
  case REC:
    if (write_rec2d_file( fname, rec ) !=0 )
      return(FALSE);
    break;
  case CARMEN:
    if (write_carmen_file( fname, rec ) !=0 )
      return(FALSE);
    break;
  case MOOS:
    if (write_moos_file( fname, rec ) !=0 )
      return(FALSE);
    break;
  case PLAYER:
    if (write_player_file( fname, rec ) !=0 )
      return(FALSE);
    break;
  case SAPHIRA:
    if (write_saphira_file( fname, rec, 45.0 ) !=0 )
      return(FALSE);
    break;
  default:
    fprintf( stderr, "# ERROR: unknown file-type!\n" );
    return(FALSE);
  }

  return(TRUE);
}

