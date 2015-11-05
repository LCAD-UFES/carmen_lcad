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

#include <carmen/carmen.h>
#include <carmen/logtools.h>

#include "defines.h"
#include "internal.h"

double round(double x);

void
rec2_parse_wifi_line( char * line, logtools_wifi_data_t * data )
{ 
  static char       dmy[MAX_LINE_LENGTH];
  static char       cmd[MAX_LINE_LENGTH];
  static char       str[MAX_LINE_LENGTH];
  static char       buf[MAX_LINE_LENGTH];
  char            * running, * ptr;
  int               ctr;
  running = line; ctr = 0;
  while ((ptr=strtok_r( ctr==0?running:NULL, ",",(char **) &buf))!=NULL) {
    sscanf( ptr, "%[^=]", str ); sscanf( str, "%s", cmd );
    if (!strncasecmp( "essid", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->essid = (char *) malloc( (strlen(str)+1)*sizeof(char) );
      strcpy( data->essid, str );
    } else if (!strncasecmp( "mode", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->mode = (char *) malloc( (strlen(str)+1)*sizeof(char) );
      strcpy( data->mode, str );
    } else if (!strncasecmp( "protocol", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->protocol = (char *) malloc( (strlen(str)+1)*sizeof(char) );
      strcpy( data->protocol, str );
    } else if (!strncasecmp( "nwid", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->nwid = atoi(str);
    } else if (!strncasecmp( "channel", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->channel = atoi(str);
    } else if (!strncasecmp( "encrypt", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->encryption = atoi(str);
    } else if (!strncasecmp( "rate", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->rate = atoi(str);
    } else if (!strncasecmp( "beacon", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->beacon = atoi(str);
    } else if (!strncasecmp( "signal", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->signal = atoi(str);
    } else if (!strncasecmp( "link", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->link = atoi(str);
    } else if (!strncasecmp( "noise", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^,]", dmy, str );
      data->noise = atoi(str);
    }
    ctr++;
  }
}
 
int
rec2_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode )
{
  static logtools_rpos2_t   npos = {0.0, 0.0, 0.0};

  static char    command[MAX_CMD_LENGTH];
  static char    dummy[MAX_CMD_LENGTH];
  static char    str1[MAX_CMD_LENGTH];
  static char    str2[MAX_CMD_LENGTH];
  static char    str3[MAX_CMD_LENGTH];
  static char    str4[MAX_CMD_LENGTH];
  static char    str5[MAX_CMD_LENGTH];
  static char    str6[MAX_CMD_LENGTH];
  static char    str7[MAX_CMD_LENGTH];
  static char    str8[MAX_CMD_LENGTH];
  static char    str9[MAX_CMD_LENGTH];
  static char    str10[MAX_CMD_LENGTH];
  static char    str11[MAX_CMD_LENGTH];
  static char    str12[MAX_CMD_LENGTH];
  static char    str13[MAX_CMD_LENGTH];
  static char    idstr[MAX_LINE_LENGTH];
  static char    datastr[MAX_LINE_LENGTH];
  static char    markerstr[MAX_LINE_LENGTH];
  static char    tagstr[MAX_LINE_LENGTH];
  static char  * running, * valptr;

  float          fov;
  double         angleDiff;
  long           sec, usec;
  int            i, l, len, nLas, nVal;

  static logtools_laser_prop2_t  lprop;
  static int                firsttime = TRUE;


  if (firsttime) {
    lprop.fov.start     =   -M_PI_2;
    lprop.fov.end       =    M_PI_2;
    lprop.fov.delta     =      M_PI;
    lprop.offset.forward  =       0.0;
    lprop.offset.forward  =       0.0;
    lprop.offset.sideward =       0.0;
    lprop.offset.rotation =       0.0;
    firsttime = FALSE;
  }

  if (strlen(line)==1 && line[0]==10 )
    return(TRUE);
	   
  if (sscanf( line, "%s", command ) == EOF) {

    return(FALSE);
	  
  }
  
  if (!strncmp( command, "POS", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s %s %s",
		dummy, &sec, &usec, str1, str2, str3, str4, str5 ) == EOF) {
      
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = POSITION;
      rec->entry[rec->numentries].index  = rec->numpositions;
      rec->numentries++;
      
      rec->psens[rec->numpositions].time.tv_sec    = sec;
      rec->psens[rec->numpositions].time.tv_usec   = usec;
      
      rec->psens[rec->numpositions].rpos.x         = atof(str1);
      rec->psens[rec->numpositions].rpos.y         = atof(str2);
      rec->psens[rec->numpositions].rpos.o         = deg2rad(atof(str3));
      
      rec->psens[rec->numpositions].rvel.tv        = atof(str4);
      rec->psens[rec->numpositions].rvel.rv        = atof(str5);

      rec->numpositions++;
    }
    
  } else if (!strncmp( command, "WIFI", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld %s",
		dummy, &sec, &usec, str1 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = WIFI;
      rec->entry[rec->numentries].index  = rec->numwifi;
      rec->numentries++;

      rec->wifi[rec->numwifi].time.tv_sec  = sec;
      rec->wifi[rec->numwifi].time.tv_usec = usec;
      len = strlen( str1 );
      rec->wifi[rec->numwifi].mac =
	(char *) malloc( (len+1) * sizeof(char) );
      strncpy( rec->wifi[rec->numwifi].mac, str1, len );

      sscanf( line, "%[^[][%[^]]", dummy, str1 );
      rec2_parse_wifi_line( str1, &(rec->wifi[rec->numwifi]) );
      rec->numwifi++;
      
    }
    
  } else if (!strncmp( command, "WIFI-DIST", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld %s %s",
		dummy, &sec, &usec, str1, str2 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = WIFI;
      rec->entry[rec->numentries].index  = rec->numwifi;
      rec->numentries++;

      rec->wifi[rec->numwifi].time.tv_sec  = sec;
      rec->wifi[rec->numwifi].time.tv_usec = usec;

      rec->wifi[rec->numwifi].precloc = TRUE;
      rec->wifi[rec->numwifi].nwid = atoi(str1);
      rec->wifi[rec->numwifi].distance = atof(str2);
      
      rec->numwifi++;
      
    }
    
  } else if (!strncmp( command, "NMEA-GGA", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s %s %s %s %s %s %s %s %s %s %s",
		dummy, &sec, &usec,
		str1, str2, str3, str4, str5, str6, str7, str8,
		str9, str10, str11, str12, str13 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = GPS;
      rec->entry[rec->numentries].index  = rec->numgps;
      rec->numentries++;
      
      rec->gps[rec->numgps].time.tv_sec  = sec;
      rec->gps[rec->numgps].time.tv_usec = usec;

      rec->gps[rec->numgps].utc              = atof(str1);
      rec->gps[rec->numgps].latitude =
	gps_degree_decimal( atof(str2), str3[0] );
      rec->gps[rec->numgps].longitude =
	gps_degree_decimal( atof(str4), str5[0] );
      rec->gps[rec->numgps].gps_quality      = atoi(str6);
      rec->gps[rec->numgps].num_satellites   = atoi(str7);
      rec->gps[rec->numgps].hdop             = atof(str8);
      rec->gps[rec->numgps].sea_level        = atof(str9);
      rec->gps[rec->numgps].altitude         = atof(str10);
      rec->gps[rec->numgps].geo_sea_level    = atof(str11);
      rec->gps[rec->numgps].geo_sep          = atof(str12);
      rec->gps[rec->numgps].data_age         = atoi(str13);
      

      rec->numgps++;
      
    }
    
    
  } else if (!strncmp( command, "MARK-POS", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %s %s %s %[^\n]",
		dummy, &sec, &usec, str1, str2, str3, str4 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = MARKER;
      rec->entry[rec->numentries].index  = rec->nummarkers;
      rec->numentries++;
      
      rec->marker[rec->nummarkers].time.tv_sec  = sec;
      rec->marker[rec->nummarkers].time.tv_usec = usec;

      if (strlen(str3)>0) {
	if (strlen(str4)>0) {
	  snprintf( str5, MAX_CMD_LENGTH,
		    "[strokecolor=black; color=white; "
		    "circle={%.3f,%.3f,120.0}; "
		    "strokecolor=black; color=red; "
		    "orientation=%.3f; arrow={%.3f,%.3f,120.0}; "
		    "orientation=0.0; fontsize=20.0; textundercolor=none; "
		    "color=black; text=%.3f,%.3f,{%s}]",
		    atof(str1), atof(str2),
		    atof(str3), atof(str1), atof(str2),
		    atof(str1)+80.0, atof(str2), str4 );
	} else {
	  snprintf( str5, MAX_CMD_LENGTH,
		    "[color=blue; circle={%.3f,%.3f}; "
		    "color=white; orientation=%.3f; arrow={%.3f,%.3f}; ",
		    atof(str1), atof(str2),
		    atof(str3), atof(str1), atof(str2) );
	}
      } else {
	snprintf( str5, MAX_CMD_LENGTH,
		  "[strokecolor=black; color=white; circle={%.3f,%.3f}]",
		  atof(str1), atof(str2) );
      }
      l = strlen(str5);
      rec->marker[rec->nummarkers].datastr =
	(char *) malloc(l * sizeof(char));
      strncpy( rec->marker[rec->nummarkers].datastr, str5, l );
      rec->nummarkers++;
  
    }
    
  } else if (!strncmp( command, "MARKER", MAX_CMD_LENGTH )) {
    
    if (sscanf( line, "%s %ld %ld: %[^\n]",
		dummy, &sec, &usec, markerstr ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = MARKER;
      rec->entry[rec->numentries].index  = rec->nummarkers;
      rec->numentries++;
      
      rec->marker[rec->nummarkers].time.tv_sec  = sec;
      rec->marker[rec->nummarkers].time.tv_usec = usec;
      
      if (sscanf( markerstr, "%[^\[] %[^\n]", dummy, datastr )==2) {
	sscanf( dummy, "%s", idstr );
	l = strlen(idstr);
	rec->marker[rec->nummarkers].tag =
	  (char *) malloc( l*sizeof(char) );
	strncpy( rec->marker[rec->nummarkers].tag, idstr, l );
	l = strlen(datastr);
	rec->marker[rec->nummarkers].datastr =
	  (char *) malloc(l * sizeof(char));
	strncpy( rec->marker[rec->nummarkers].datastr, datastr, l );
      } else {
	rec->marker[rec->nummarkers].tag =
	  (char *) malloc( (strlen(NO_TAG)+1)*sizeof(char) );
	strcpy( rec->marker[rec->nummarkers].tag, NO_TAG );
	l = strlen(markerstr);
	rec->marker[rec->nummarkers].datastr =
	  (char *) malloc(l * sizeof(char));
	strncpy( rec->marker[rec->nummarkers].datastr, markerstr, l );
      }

      rec->nummarkers++;
      
    }
    
  } else if (!strcmp( command, "CARMEN-LASER") ){
    
    if (sscanf( line, "%s %ld %ld %d %d %f:",
		dummy, &sec, &usec, &nLas, &nVal, &fov ) == EOF) {
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = nLas;
      rec->lsens[rec->numlaserscans].laser.time.tv_sec  = sec;
      rec->lsens[rec->numlaserscans].laser.time.tv_usec = usec;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;
      rec->lsens[rec->numlaserscans].coord              = NULL;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (float *) malloc( nVal * sizeof(float) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (float *) malloc( nVal * sizeof(float) );
      }
      rec->lsens[rec->numlaserscans].laser.fov = deg2rad(fov);
      if (fabs(rad2deg(lprop.fov.delta)-fov)>DEFAULT_EPSILON) {
	lprop.fov.delta = deg2rad(fov);
	lprop.fov.start = -(lprop.fov.delta/2.0);
	lprop.fov.end   = (lprop.fov.delta/2.0);
	
      }
      angleDiff = lprop.fov.delta / (double) (nVal-1);
      rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;
      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    100.0 * atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.fov.start+(i*angleDiff);
	}
      }
      if (rec->numpositions>0)
	rec->lsens[rec->numlaserscans].estpos =
	  rec->psens[rec->numpositions-1].rpos;
      else
	rec->lsens[rec->numlaserscans].estpos = npos;
      
      rec->numlaserscans++;
    }
    
  } else if (!strcmp( command, "LASER-RANGE") ||
	     !strcmp( command, "LASER-SCAN") ){
    
    if (sscanf( line, "%s %ld %ld %d %d %f%[^:]:",
	       dummy, &sec, &usec, &nLas, &nVal, &fov, tagstr ) == EOF) {
      return(FALSE);
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = nLas;
      rec->lsens[rec->numlaserscans].laser.time.tv_sec  = sec;
      rec->lsens[rec->numlaserscans].laser.time.tv_usec = usec;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;
      rec->lsens[rec->numlaserscans].coord              = NULL;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (float *) malloc( nVal * sizeof(float) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (float *) malloc( nVal * sizeof(float) );
      }
      rec->lsens[rec->numlaserscans].laser.fov = deg2rad(fov);
      if (fabs(rad2deg(lprop.fov.delta)-fov)>DEFAULT_EPSILON) {
	lprop.fov.delta = deg2rad(fov);
	lprop.fov.start = -(lprop.fov.delta/2.0);
	lprop.fov.end   = (lprop.fov.delta/2.0);
	
      }
      angleDiff = lprop.fov.delta / (double) (nVal-1);
      rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;

      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.fov.start+(i*angleDiff);
	}
      }
      if (rec->numpositions>0)
	rec->lsens[rec->numlaserscans].estpos =
	  rec->psens[rec->numpositions-1].rpos;
      else
	rec->lsens[rec->numlaserscans].estpos = npos;
      
      rec->numlaserscans++;
    }
    
  } else if (!strcmp( command, "LASER") ){
    
    if (sscanf( line, "%s %ld %ld %d %d:",
		dummy, &sec, &usec, &nLas, &nVal ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = nLas;
      rec->lsens[rec->numlaserscans].laser.time.tv_sec  = sec;
      rec->lsens[rec->numlaserscans].laser.time.tv_usec = usec;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (float *) malloc( nVal * sizeof(float) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (float *) malloc( nVal * sizeof(float) );
      }
      
      rec->lsens[rec->numlaserscans].coord             = NULL;
      
      if (nVal==360 || nVal==180)
	angleDiff = lprop.fov.delta / (double) (nVal);
      else
	angleDiff = lprop.fov.delta / (double) (nVal-1);
      rec->lsens[rec->numlaserscans].laser.fov    = lprop.fov.delta;
      rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;
      
      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.val[i] =
	    atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]      =
	    lprop.fov.start + i*angleDiff;
	}
      }
      if (rec->numpositions>0)
	rec->lsens[rec->numlaserscans].estpos =
	  rec->psens[rec->numpositions-1].rpos;
      else
	rec->lsens[rec->numlaserscans].estpos = npos;
      
      rec->numlaserscans++;
      
    }
    

  } else {
    
    if (!(command[0]=='#' || command[0]=='*')) {
      if (!(mode && READ_MODE_DONT_STOP)) {
	fprintf( stderr, "ERROR: unknown keyword %s\n", command );
	return(FALSE);
      }
    }
    
  }

  return(TRUE);
}

int
carmen_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode )
{
  static char    markerstr[MAX_LINE_LENGTH];
  static char    command[MAX_CMD_LENGTH];
  static char    dummy[MAX_CMD_LENGTH];
  static char    str1[MAX_CMD_LENGTH];
  static char    str2[MAX_CMD_LENGTH];
  static char    str3[MAX_CMD_LENGTH];
  static char    str4[MAX_CMD_LENGTH];
  static char    str5[MAX_CMD_LENGTH];
  static char    str6[MAX_CMD_LENGTH];
  static char    str7[MAX_CMD_LENGTH];
  static char    str8[MAX_CMD_LENGTH];
  static char  * running, * valptr;

  float          fov = M_PI;
  double         angleDiff, time;
  int            i, l, nVal, numRemiss;

  static logtools_laser_prop2_t  lprop;
  static int                firsttime = TRUE;

  carmen_robot_laser_message cl;
  carmen_erase_structure(&cl, sizeof(carmen_robot_laser_message) );

  if (firsttime) {
    lprop.fov.start     =   -M_PI_2;
    lprop.fov.end       =    M_PI_2;
    lprop.fov.delta     =      M_PI;
    lprop.offset.forward  =       0.0;
    lprop.offset.forward  =       0.0;
    lprop.offset.sideward =       0.0;
    lprop.offset.rotation =       0.0;
    firsttime = FALSE;
  }

  if (strlen(line)==1 && line[0]==10 )
    return(TRUE);
	   
  if (sscanf( line, "%s", command ) == EOF) {

    return(FALSE);
	  
  }
  
  if (!strcmp( command, "ODOM") ){
    
    if (sscanf( line, "%s %s %s %s %s %s %s %s",
		dummy, str1, str2, str3, str4, str5, str6, str7 ) == EOF) {
      
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = POSITION;
      rec->entry[rec->numentries].index  = rec->numpositions;
      rec->numentries++;
      
      rec->psens[rec->numpositions].rpos.x         = atof(str1)*100.0;
      rec->psens[rec->numpositions].rpos.y         = atof(str2)*100.0;
      rec->psens[rec->numpositions].rpos.o         = atof(str3);
      
      rec->psens[rec->numpositions].rvel.tv        = atof(str4);
      rec->psens[rec->numpositions].rvel.rv        = atof(str5);
 
      time = atof(str7);
      convert_time( time, &rec->psens[rec->numpositions].time );
      rec->numpositions++;
    }
    
    
  } else if (!strcmp( command, "RAWLASER1") ){
    
    if (sscanf( line, "%s %s %s %s %s %s %s %s %s",
		dummy, str1, str2, str3, str4,
		str5, str6, str7, str8 ) == EOF) {
		
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;

      nVal = atoi(str8);
      rec->lsens[rec->numlaserscans].id                 = 10;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;
      rec->lsens[rec->numlaserscans].coord              = NULL;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (float *) malloc( nVal * sizeof(float) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (float *) malloc( nVal * sizeof(float) );
      }
      if (fabs(rad2deg(lprop.fov.delta)-fov)>DEFAULT_EPSILON) {
	lprop.fov.delta = atof(str3);
	lprop.fov.start = atof(str2);
	lprop.fov.end   = lprop.fov.delta + lprop.fov.start;
      }
      rec->lsens[rec->numlaserscans].laser.fov = lprop.fov.delta;
      rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;
      angleDiff = atof(str4);

      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    100.0 * atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.fov.start+(i*angleDiff);
	}
      }
      valptr = strtok( NULL, " ");
      numRemiss = atoi(valptr);
      for (i=0;i<numRemiss;i++) {
	valptr = strtok( NULL, " ");
      }
      if (rec->numpositions>0)
	rec->lsens[rec->numlaserscans].estpos =
	  rec->psens[rec->numpositions-1].rpos;
      valptr = strtok( NULL, " ");
      time = atof(valptr);
      convert_time( time, &rec->lsens[rec->numlaserscans].laser.time );
      rec->numlaserscans++;
    }
    
  } else if (!strcmp( command, "FLASER") ){
    
    if (sscanf( line, "%s %d ",
		dummy, &nVal ) == EOF) {
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = 0;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;
      rec->lsens[rec->numlaserscans].coord              = NULL;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (float *) malloc( nVal * sizeof(float) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (float *) malloc( nVal * sizeof(float) );
      }
      if (fabs(rad2deg(lprop.fov.delta)-fov)>DEFAULT_EPSILON) {
	lprop.fov.delta = M_PI;
	lprop.fov.start = -(lprop.fov.delta/2.0);
	lprop.fov.end   = (lprop.fov.delta/2.0);
      }
      rec->lsens[rec->numlaserscans].laser.fov = lprop.fov.delta;
      rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;
      angleDiff = lprop.fov.delta / (double) (nVal-1);

      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    100.0 * atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.fov.start+(i*angleDiff);
	}
      }
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.x = 100.0 * atof(valptr);
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.y = 100.0 * atof(valptr);
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.o = atof(valptr);
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      valptr = strtok( NULL, " ");
      time = atof(valptr);
      convert_time( time, &rec->lsens[rec->numlaserscans].laser.time );
      rec->numlaserscans++;
    }
    
  } else if (!strcmp( command, "ROBOTLASER1") ){

    carmen_string_to_robot_laser_message(line, &cl);
    nVal = cl.num_readings;

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
      
      rec->lsens[rec->numlaserscans].id                 = 0;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;
      rec->lsens[rec->numlaserscans].coord              = NULL;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (float *) malloc( nVal * sizeof(float) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (float *) malloc( nVal * sizeof(float) );
      }
      if (fabs(rad2deg(lprop.fov.delta)-fov)>DEFAULT_EPSILON) {
	lprop.fov.delta = cl.config.fov;
	lprop.fov.start = cl.config.start_angle;
	lprop.fov.end   = cl.config.start_angle + cl.config.fov;
      }
      rec->lsens[rec->numlaserscans].laser.fov = lprop.fov.delta;
      rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;
      angleDiff = cl.config.angular_resolution;
      
      for (i=0;i<nVal;i++) {
	rec->lsens[rec->numlaserscans].laser.val[i] = 100.0 * cl.range[i];
	rec->lsens[rec->numlaserscans].laser.angle[i]  = lprop.fov.start+(i*angleDiff);
      }
      rec->lsens[rec->numlaserscans].estpos.x = 100.0 * cl.laser_pose.x;
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.y = 100.0 * cl.laser_pose.y;
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.o = cl.laser_pose.theta;
      time = cl.timestamp;
      convert_time( time, &rec->lsens[rec->numlaserscans].laser.time );
      rec->numlaserscans++;
  } else if (!strcmp( command, "ROBOTLASER2") ){

    carmen_string_to_robot_laser_message(line, &cl);
    nVal = cl.num_readings;

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
      
      rec->lsens[rec->numlaserscans].id                 = 1;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;
      rec->lsens[rec->numlaserscans].coord              = NULL;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (float *) malloc( nVal * sizeof(float) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (float *) malloc( nVal * sizeof(float) );
      }
      if (fabs(rad2deg(lprop.fov.delta)-fov)>DEFAULT_EPSILON) {
	lprop.fov.delta = cl.config.fov;
	lprop.fov.start = cl.config.start_angle;
	lprop.fov.end   = cl.config.start_angle + cl.config.fov;
      }
      rec->lsens[rec->numlaserscans].laser.fov = lprop.fov.delta;
      rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;
      angleDiff = cl.config.angular_resolution;
      
      for (i=0;i<nVal;i++) {
	rec->lsens[rec->numlaserscans].laser.val[i] = 100.0 * cl.range[i];
	rec->lsens[rec->numlaserscans].laser.angle[i]  = lprop.fov.start+(i*angleDiff);
      }
      rec->lsens[rec->numlaserscans].estpos.x = 100.0 * cl.laser_pose.x;
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.y = 100.0 * cl.laser_pose.y;
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.o = cl.laser_pose.theta;
      time = cl.timestamp;
      convert_time( time, &rec->lsens[rec->numlaserscans].laser.time );
      rec->numlaserscans++;
  } else if (!strcmp( command, "RLASER") ){
    
    if (sscanf( line, "%s %d ",
		dummy, &nVal ) == EOF) {
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      rec->lsens[rec->numlaserscans].id                 = 1;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;

      rec->lsens[rec->numlaserscans].coord              = NULL;

      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (float *) malloc( nVal * sizeof(float) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (float *) malloc( nVal * sizeof(float) );
      }
      rec->lsens[rec->numlaserscans].laser.fov = M_PI;
      if (fabs(rad2deg(lprop.fov.delta)-fov)>DEFAULT_EPSILON) {
	lprop.fov.delta = M_PI;
	lprop.fov.start = -(lprop.fov.delta/2.0);
	lprop.fov.end   = (lprop.fov.delta/2.0);
      }
      angleDiff = lprop.fov.delta / (double) (nVal-1);
      rec->lsens[rec->numlaserscans].laser.fov = lprop.fov.delta;
      rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;

      running = line;
      strtok( running, " ");
      strtok( NULL, " ");
      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    100.0 * atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.fov.start+(i*angleDiff);
	}
      }
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.x = 100.0 * atof(valptr);
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.y = 100.0 * atof(valptr);
      valptr = strtok( NULL, " ");
      rec->lsens[rec->numlaserscans].estpos.o = atof(valptr);
      strtok( NULL, " ");
      strtok( NULL, " ");
      strtok( NULL, " ");
      valptr = strtok( NULL, " ");
      time = atof(valptr);
      convert_time( time, &rec->lsens[rec->numlaserscans].laser.time );
      rec->numlaserscans++;
    }
    
  } else if (!strcmp( command, "MARKER" )) {
    
    if (sscanf( line, "%s %[^\n] %s",
		dummy, markerstr, str1 ) == EOF) {
      
      return(FALSE);
      
    } else {
      
      rec->entry[rec->numentries].type   = MARKER;
      rec->entry[rec->numentries].index  = rec->nummarkers;
      rec->numentries++;
      
      l = strlen(markerstr);
      rec->marker[rec->nummarkers].datastr =
	(char *) malloc(l * sizeof(char));
      strncpy( rec->marker[rec->nummarkers].datastr, markerstr, l );

      time = atof(str1);
      convert_time( time, &rec->marker[rec->nummarkers].time );
      
      rec->nummarkers++;
      
    }
    
  } else {
    
    if (!(command[0]=='#' || command[0]=='*')) {
      if (!(mode && READ_MODE_DONT_STOP)) {
	fprintf( stderr, "ERROR: unknown keyword %s\n", command );
	return(FALSE);
      }
    }
    
  }

  return(TRUE);
}

int
moos_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode )
{
  static logtools_rpos2_t   npos = {0.0, 0.0, 0.0};
  static char    command[MAX_CMD_LENGTH];
  static char    dummy[MAX_CMD_LENGTH];
  static char  * running, * valptr, * strptr, * datastr, * laserstr;

  double         angleDiff, time, fov;
  int            i, nVal;

  static logtools_laser_prop2_t  lprop;
  static int                firsttime = TRUE;


  if (firsttime) {
    lprop.fov.start     =   -M_PI_2;
    lprop.fov.end       =    M_PI_2;
    lprop.fov.delta     =      M_PI;
    lprop.offset.forward  =       0.0;
    lprop.offset.forward  =       0.0;
    lprop.offset.sideward =       0.0;
    lprop.offset.rotation =       0.0;
    firsttime = FALSE;
  }

  if (strlen(line)==1 && line[0]==10 )
    return(TRUE);

  if (sscanf( line, "%s %s", dummy, command ) == EOF) {

    return(FALSE);
	  
  }
  
  if (!strncmp( command, "ROBOT_POSITION", MAX_CMD_LENGTH )) {

    datastr = strstr( line, "time" );

    if ( datastr == NULL ) {
	
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = POSITION;
      rec->entry[rec->numentries].index  = rec->numpositions;
      rec->numentries++;
      
      running = datastr;

      strptr = index(strtok( running, ","),'=');
      time = atof(++strptr);
      convert_time( time, &rec->psens[rec->numpositions].time );
      
      strptr = index(strtok( NULL, ","),'=');
      rec->psens[rec->numpositions].rpos.x         = atof(++strptr)*100.0;

      strptr = index(strtok( NULL, ","),'=');
      rec->psens[rec->numpositions].rpos.y         = atof(++strptr)*100.0;

      strptr = index(strtok( NULL, ","),'=');
      rec->psens[rec->numpositions].rpos.o         = deg2rad(90.0+atof(++strptr));
      
      rec->psens[rec->numpositions].rvel.tv        = 0.0;
      rec->psens[rec->numpositions].rvel.rv        = 0.0;

      rec->numpositions++;

    }
    
  } else if (!strcmp( command, "LASER_RANGE") ){
     
    laserstr = strstr( line, "range" );
    datastr  = strstr( line, "time" );

    if ( datastr == NULL ) {
	
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = LASER_VALUES;
      rec->entry[rec->numentries].index  = rec->numlaserscans;
      rec->numentries++;
	  
      running = datastr;
      strptr = index(strtok( running, ","),'=');
      time = atof(++strptr); 
      convert_time( time, &rec->lsens[rec->numlaserscans].laser.time );

      running = laserstr;
      strptr = strtok( running, "{");
      sscanf( strptr, "range[%d]", &nVal );

      rec->lsens[rec->numlaserscans].id                 = 0;
      rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;

      rec->lsens[rec->numlaserscans].coord              = NULL;
      
      if (alloc) {
	rec->lsens[rec->numlaserscans].laser.val =
	  (float *) malloc( nVal * sizeof(float) );
	rec->lsens[rec->numlaserscans].laser.angle =
	  (float *) malloc( nVal * sizeof(float) );
      }

      fov = sick_laser_fov(nVal);
      rec->lsens[rec->numlaserscans].laser.fov = fov;
      lprop.fov.delta = fov;
      lprop.fov.start = -(lprop.fov.delta/2.0);
      lprop.fov.end   = (lprop.fov.delta/2.0);
      angleDiff = sick_laser_angle_diff( nVal, fov );
      rec->lsens[rec->numlaserscans].laser.fov = lprop.fov.delta;
      rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;

      for (i=0;i<nVal;i++) {
	valptr = strtok( NULL, " ");
	if (valptr==NULL) {
	  return(FALSE);
	} else {
	  rec->lsens[rec->numlaserscans].laser.val[i]       =
	    100.0 * atof(valptr);
	  rec->lsens[rec->numlaserscans].laser.angle[i]     =
	    lprop.fov.start+(i*angleDiff);
	}
      }

      if (rec->numpositions>0)
	rec->lsens[rec->numlaserscans].estpos =
	  rec->psens[rec->numpositions-1].rpos;
      else
	rec->lsens[rec->numlaserscans].estpos = npos;

      rec->numlaserscans++;
    }
    
  } else {
    
    if (!(command[0]=='#' || command[0]=='*')) {
      if (!(mode && READ_MODE_DONT_STOP)) {
	fprintf( stderr, "ERROR: unknown keyword %s\n", command );
	return(FALSE);
      }
    }
    
  }

  return(TRUE);
}

enum PLAYER_COMMANDS { LASER, POS, OTHER };

int
player_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode )
{
  static logtools_rpos2_t   npos = {0.0, 0.0, 0.0};
  static char    dummy[MAX_CMD_LENGTH];
  static char  * running, * valptr;
  double         angleDiff, time, fov;
  int            i, nVal;
  enum PLAYER_COMMANDS command;
  
  static logtools_laser_prop2_t  lprop;
  static int                firsttime = TRUE;
  static float              laser[MAX_NUM_LASER_BEAMS];

  if (firsttime) {
    lprop.fov.start     =   -M_PI_2;
    lprop.fov.end       =    M_PI_2;
    lprop.fov.delta     =      M_PI;
    lprop.offset.forward  =       0.0;
    lprop.offset.forward  =       0.0;
    lprop.offset.sideward =       0.0;
    lprop.offset.rotation =       0.0;
    firsttime = FALSE;
  }

  if ( strlen(line)==1 && line[0]==10 )
    return(TRUE);
 
  if ( strstr( line, "position" ) != NULL ) {
    command = POS;
  } else if ( strstr( line, "laser" ) != NULL ) {
    command = LASER;
  } else {
    command = OTHER;
  }
 
  switch( command ) {

  case POS:

    rec->entry[rec->numentries].type   = POSITION;
    rec->entry[rec->numentries].index  = rec->numpositions;
    rec->numentries++;
    
    running = strstr( line, "position" );

    strtok( running, " ");
    strtok( NULL, " ");

    time = atof(strtok( NULL, " "));
    convert_time( time, &rec->psens[rec->numpositions].time );
    
    rec->psens[rec->numpositions].rpos.x  = atof(strtok( NULL, " "))*100.0;
    rec->psens[rec->numpositions].rpos.y  = atof(strtok( NULL, " "))*100.0;
    rec->psens[rec->numpositions].rpos.o  = atof(strtok( NULL, " ")); 
    
    rec->psens[rec->numpositions].rvel.tv        = 0.0;
    rec->psens[rec->numpositions].rvel.rv        = 0.0;
    
    rec->numpositions++;

    break;
    
  case LASER:

    rec->entry[rec->numentries].type   = LASER_VALUES;
    rec->entry[rec->numentries].index  = rec->numlaserscans;
    rec->numentries++;
	  
    running = strstr( line, "laser" );

    strtok( running, " ");
    rec->lsens[rec->numlaserscans].id = (int) atoi( strtok( NULL, " ") );

    time = atof(strtok( NULL, " "));
    convert_time( time, &rec->psens[rec->numpositions].time );

    nVal = 0;
    while ( (valptr = strtok( NULL, " "))!=NULL) {
      laser[nVal++] = atof(valptr) * 100.0;
      strtok( NULL, " ");
      strtok( NULL, " ");
    } 
    nVal--;
    
    rec->lsens[rec->numlaserscans].laser.numvalues    = nVal;
    rec->lsens[rec->numlaserscans].coord              = NULL;
    
    if (alloc) {
      rec->lsens[rec->numlaserscans].laser.val =
	(float *) malloc( nVal * sizeof(float) );
      rec->lsens[rec->numlaserscans].laser.angle =
	(float *) malloc( nVal * sizeof(float) );
    }

    fov = sick_laser_fov(nVal);
    rec->lsens[rec->numlaserscans].laser.fov = fov;
    rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;
    lprop.fov.delta = fov;
    lprop.fov.start = -(lprop.fov.delta/2.0);
    lprop.fov.end   = (lprop.fov.delta/2.0);
    angleDiff = sick_laser_angle_diff( nVal, fov );

    for (i=0;i<nVal;i++) {
      rec->lsens[rec->numlaserscans].laser.val[i]       = laser[i];
      rec->lsens[rec->numlaserscans].laser.angle[i]     =
	lprop.fov.start+(i*angleDiff);
    }
    
    if (rec->numpositions>0)
      rec->lsens[rec->numlaserscans].estpos =
	rec->psens[rec->numpositions-1].rpos;
    else
      rec->lsens[rec->numlaserscans].estpos = npos;
    
    rec->numlaserscans++;

    break;
    
	
  default:

    if (sscanf( line, "%s", dummy ) == EOF) {
      return(FALSE);
    }
    if (!(dummy[0]=='#' || dummy[0]=='*')) {
      if (!(mode && READ_MODE_DONT_STOP)) {
	fprintf( stderr, "ERROR: unknown keyword %s\n", dummy );
	return(FALSE);
      }
    }
    break;
    
  }
  
  return(TRUE);
}

int
saphira_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode )
{
  static logtools_rpos2_t   npos = {0.0, 0.0, 0.0};
  static char    command[MAX_CMD_LENGTH];
  static char    dummy[MAX_CMD_LENGTH];
  static char    str1[MAX_CMD_LENGTH];
  static char    str2[MAX_CMD_LENGTH];
  static char    str3[MAX_CMD_LENGTH];
  static char  * running, * ptr;

  double         angleDiff;
  int            i, v;

  static logtools_laser_prop2_t  lprop;
  static int                firsttime = TRUE;
  static int                cur_robot_id;
  static int                laser_num_values = 181;
  static float              laser[MAX_NUM_LASER_BEAMS];
  static int                warned = FALSE;
  
  if (firsttime) {
    lprop.fov.start     =   -M_PI_2;
    lprop.fov.end       =    M_PI_2;
    lprop.fov.delta     =      M_PI;
    lprop.offset.forward  =       0.0;
    lprop.offset.forward  =       0.0;
    lprop.offset.sideward =       0.0;
    lprop.offset.rotation =       0.0;
    firsttime = FALSE;
  }

  if (strlen(line)==1 && line[0]==10 )
    return(TRUE);
	   
  if (sscanf( line, "%s", command ) == EOF) {

    return(FALSE);
	  
  }
  
  if (!strcmp( command, "robot_id:") ){

    sscanf(line, "%s %d", dummy, &cur_robot_id );
    
  } else if (!strcmp( command, "sick1conf:") ){

    if (sscanf( line, "%s %s %s %s",
		dummy, str1, str2, str3 ) == EOF) {
      
      return(FALSE);
      
    } else {

      lprop.fov.start = atof(str1);
      lprop.fov.end   = atof(str2);
      lprop.fov.delta = lprop.fov.end-lprop.fov.start;
      laser_num_values  = (int) atoi(str3);
    }
    
  } else if (!strcmp( command, "robot:") ){
    
    if (sscanf( line, "%s %s %s %s",
		dummy, str1, str2, str3 ) == EOF) {
      
      return(FALSE);
      
    } else {

      rec->entry[rec->numentries].type   = POSITION;
      rec->entry[rec->numentries].index  = rec->numpositions;
      rec->numentries++;
      
      rec->psens[rec->numpositions].rpos.x         = atof(str1)/10.0;
      rec->psens[rec->numpositions].rpos.y         = atof(str2)/10.0;
      rec->psens[rec->numpositions].rpos.o         = deg2rad(atof(str3));
      
      rec->psens[rec->numpositions].rvel.tv        = 0.0;
      rec->psens[rec->numpositions].rvel.rv        = 0.0;
 
      rec->psens[rec->numpositions].time.tv_sec    = 0;
      rec->psens[rec->numpositions].time.tv_usec    = 0;

      rec->numpositions++;
    }
    
    
  } else if (!strcmp( command, "sick1:") ){
    
    running = line;
    strtok( running, " ");
    v=0;
    while ((ptr=strtok( NULL, " "))!=NULL) {
      laser[v++] = round(atof(ptr)/10.0);
    }
    v--;
    
    if (v!=laser_num_values && !warned) {
      warned = TRUE;
      fprintf( stderr,
	       "* WARNING:"
	       " number of laser values (%d) does not fit with sick1conf (%d)!\n", v, laser_num_values);
    }

    rec->entry[rec->numentries].type   = LASER_VALUES;
    rec->entry[rec->numentries].index  = rec->numlaserscans;
    rec->numentries++;
	  
    rec->lsens[rec->numlaserscans].id                 = 0;
    rec->lsens[rec->numlaserscans].laser.numvalues    = v;

    rec->lsens[rec->numlaserscans].coord              = NULL;

    if (alloc) {
      rec->lsens[rec->numlaserscans].laser.val =
	(float *) malloc( v * sizeof(float) );
      rec->lsens[rec->numlaserscans].laser.angle =
	(float *) malloc( v * sizeof(float) );
    }
    rec->lsens[rec->numlaserscans].laser.fov = sick_laser_fov(v);
    if (fabs( rad2deg(lprop.fov.delta)-
	      rec->lsens[rec->numlaserscans].laser.fov) > DEFAULT_EPSILON) {
      lprop.fov.delta = sick_laser_fov(v);
      lprop.fov.start = -(lprop.fov.delta/2.0);
      lprop.fov.end   = (lprop.fov.delta/2.0);
    }
    angleDiff =
      sick_laser_angle_diff( v, rec->lsens[rec->numlaserscans].laser.fov );
    rec->lsens[rec->numlaserscans].laser.offset = lprop.offset;

    for (i=0;i<v;i++) {
      rec->lsens[rec->numlaserscans].laser.val[i]       = laser[i];
      rec->lsens[rec->numlaserscans].laser.angle[i]     =
	lprop.fov.start+(i*angleDiff);
    }
    rec->lsens[rec->numlaserscans].laser.time.tv_sec   = 0;
    rec->lsens[rec->numlaserscans].laser.time.tv_usec  = 0;

    if (rec->numpositions>0)
      rec->lsens[rec->numlaserscans].estpos =
	rec->psens[rec->numpositions-1].rpos;
    else
      rec->lsens[rec->numlaserscans].estpos = npos;
    
    rec->numlaserscans++;
    
    
  } else {
    
    if (!(command[0]=='#' || command[0]=='*')) {
      if (!(mode && READ_MODE_DONT_STOP)) {
	fprintf( stderr, "ERROR: unknown keyword %s\n", command );
	return(FALSE);
      }
    }
    
  }

  return(TRUE);
}

void
placelab_parse_marker_line( char * line, logtools_marker_data_t * data ){
  static char       dmy[MAX_LINE_LENGTH];
  static char       cmd[MAX_LINE_LENGTH];
  static char       str[MAX_LINE_LENGTH];
  static char       buf[MAX_LINE_LENGTH];
  char            * running, * ptr;
  int               l, ctr;
  double            tf;
  running = line; ctr = 0;
  while ((ptr=strtok_r( ctr==0?running:NULL, "|",(char **) &buf))!=NULL) {
    sscanf( ptr, "%[^=]", str ); sscanf( str, "%s", cmd );
    if (!strncasecmp( "type", cmd, MAX_LINE_LENGTH)) {
    } else if (!strncasecmp( "time", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      tf = atof(str);
      data->time.tv_sec = (int) (tf/1000.0);
      data->time.tv_usec = (int) ((tf-(data->time.tv_sec*1000.0))*1000.0);
    } else if (!strncasecmp( "tag", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      l = strlen(str);
      data->tag = (char *) malloc((l+1) * sizeof(char));
      strncpy( data->tag, str, l );
    } else if (!strncasecmp( "data", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      l = strlen(str);
      data->datastr = (char *) malloc((l+1) * sizeof(char));
      strncpy( data->datastr, str, l );
    }
    ctr++;
  }
}

void
placelab_parse_gps_line( char * line, logtools_gps_data_t * data )
{ 
  static char       dmy[MAX_LINE_LENGTH];
  static char       cmd[MAX_LINE_LENGTH];
  static char       str[MAX_LINE_LENGTH];
  static char       buf[MAX_LINE_LENGTH];
  char            * running, * ptr;
  int               ctr;
  double            tf;
  running = line; ctr = 0;
  while ((ptr=strtok_r( ctr==0?running:NULL, "|",(char **) &buf))!=NULL) {
    sscanf( ptr, "%[^=]", str ); sscanf( str, "%s", cmd );
    if (!strncasecmp( "type", cmd, MAX_LINE_LENGTH)) {
    } else if (!strncasecmp( "time", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      tf = atof(str);
      data->time.tv_sec = (int) (tf/1000.0);
      data->time.tv_usec = (int) ((tf-(data->time.tv_sec*1000.0))*1000.0);
    } else if (!strncasecmp( "lat", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      data->latitude = atof(str);
    } else if (!strncasecmp( "lon", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      data->longitude = atof(str);
    } else if (!strncasecmp( "quality", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      data->gps_quality = atoi(str);
    } else if (!strncasecmp( "numsat", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      data->num_satellites = atoi(str);
    } else if (!strncasecmp( "geoheight", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      data->sea_level = atof(str);
    } else if (!strncasecmp( "antheight", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      data->altitude = atof(str);
    } else if (!strncasecmp( "timeoffix", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      data->utc = atof(str);
    } else if (!strncasecmp( "hdop", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      data->hdop = atof(str);
    } else if (!strncasecmp( "dgpsid", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
    } else if (!strncasecmp( "cog", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
    } else if (!strncasecmp( "dgpsusage", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
    } else if (!strncasecmp( "mode", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
    } else if (!strncasecmp( "date", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
    } else if (!strncasecmp( "var", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
    } else if (!strncasecmp( "vardir", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
    } else if (!strncasecmp( "sog", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      data->speed_over_ground = atof(str);
    } else if (!strncasecmp( "status", cmd, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dmy, str );
      if (strncasecmp( str, "V", 1 ))
	data->status = 1;
      else
	data->status = 0;
    }
    ctr++;
  }
}

int
placelab_parse_line( char *line, logtools_log_data_t *rec,
		     int alloc __attribute__ ((unused)), int mode )
{
  char  * ptr, * running;
  int     ctr;
  static char    cline[MAX_LINE_LENGTH];
  static char    buf[MAX_LINE_LENGTH];
  static char    command[MAX_CMD_LENGTH];
  static char    dummy[MAX_CMD_LENGTH];
  static int     continuous = TRUE;
  
  running = line;
  ctr = 0;
  rec->entry[rec->numentries].type   = UNKNOWN;
  strcpy( cline, line );
  while ((ptr=strtok_r( ctr==0?running:NULL, "|",(char **) &buf))!=NULL) {
    sscanf( ptr, "%[^=]", command );
    if (!strncasecmp( "type", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^|]", dummy, command );
      if (!strncasecmp( "gps", command, MAX_LINE_LENGTH)) {
	rec->entry[rec->numentries].type   = GPS;
	rec->entry[rec->numentries].index  = rec->numgps;
	break;
      } else if (!strncasecmp( "marker", command, MAX_LINE_LENGTH)) {
	rec->entry[rec->numentries].type   = MARKER;
	rec->entry[rec->numentries].index  = rec->nummarkers;
	break;
      } else {
      }
    }
    ctr++;
  }
  if (rec->entry[rec->numentries].type==UNKNOWN) {
    if (sscanf( cline, "%s", command ) == EOF) {
      return(TRUE);
    }
    if (!(command[0]=='#' || command[0]=='*')) {
      if (!(mode && READ_MODE_DONT_STOP)) {
	fprintf( stderr, "ERROR: unknown line %s\n", cline );
	return(TRUE);
      }
    }
    return(TRUE);
  } else {
    switch (rec->entry[rec->numentries].type) {
    case MARKER:
      placelab_parse_marker_line( cline, &(rec->marker[rec->nummarkers]) );
      rec->nummarkers++;
      rec->numentries++;
      return(TRUE);
      break;
    case GPS:
      placelab_parse_gps_line( cline, &(rec->gps[rec->numgps]) );
      rec->numgps++;
      rec->numentries++;
      continuous = FALSE;
      return(TRUE);
      break;
    default:
      return(TRUE);
      break;
    }
  }
  return(TRUE);
}
