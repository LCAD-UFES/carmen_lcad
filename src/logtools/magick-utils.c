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
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include <wand/magick_wand.h>

#include <carmen/logtools.h>
#include <carmen/logtools_graphics.h>

#include "log2pic.h"

// You should not use LCAD version of Carmen with Fedora 14. Do not uncomment the line below for compiling the LCAD Carmen in Fedora 14. Alberto.
//#define DestroyConstitute ConstituteComponentTerminus

PointInfo arrow[7] = {
  { -1.0, -0.15 }, 
  {  0.2, -0.15 },
  {  0.2, -0.60 },
  {  1.0,  0.00 },
  {  0.2,  0.60 },
  {  0.2,  0.15 },
  { -1.0,  0.15 }
};

double
metric_to_pixel( double size, logtools_grid_map2_t * map,
		 enum logtools_file_t system )
{
  if (system==CARMEN) {
    return( 100.0 * size / (double) map->resolution );
  } else {
    return( size / (double) map->resolution );
  }
}

int
marker_map_pos_from_robot_pos( logtools_rpos2_t rpos, logtools_grid_map2_t * map,
			       logtools_vector2_t * iv,
			       enum logtools_file_t system )
{
  logtools_vector2_t pos, pos_r, v, vr;
  int     ret;
  if (system==CARMEN) {
    pos.x = rpos.x * 100.0;
    pos.y = rpos.y * 100.0;
  } else {
    pos.x = rpos.x;
    pos.y = rpos.y;
  }
  if (fabs(settings.rotation_angle)>MIN_ROTATION) {
    vr.x = pos.x - settings.rotation_center.x;
    vr.y = pos.y - settings.rotation_center.y;
    pos_r = logtools_rotate_vector2( vr, settings.rotation_angle );
    pos.x = pos_r.x + settings.rotation_center.x;
    pos.y = pos_r.y + settings.rotation_center.y;
  }
  ret = log2pic_map_pos_from_vec2( pos, map, &v );
  iv->y = (map->zoom*(map->mapsize.y-v.y-1));
  iv->x = (map->zoom*v.x);
  return(ret);
}

int
magick_pos_without_rotation( logtools_rpos2_t rpos,
			     logtools_grid_map2_t * map,
			     logtools_vector2_t * iv )
{
  logtools_vector2_t pos, v;
  int     ret;
  pos.x = rpos.x;
  pos.y = rpos.y;
  ret = log2pic_map_pos_from_vec2( pos, map, &v );
  iv->y = (map->zoom*(map->mapsize.y-v.y-1));
  iv->x = (map->zoom*v.x);
  return(ret);
}

int
magick_pos_from_vector( logtools_vector2_t pos, logtools_grid_map2_t * map,
			logtools_vector2_t * iv )
{
  logtools_vector2_t v;
  int     ret;
  ret = log2pic_map_pos_from_vec2( pos, map, &v );
  iv->y = (map->zoom*(map->mapsize.y-v.y-1));
  iv->x = (map->zoom*v.x);
  return(ret);
}

int
magick_pos_from_rpos( logtools_rpos2_t rpos, logtools_grid_map2_t * map, logtools_vector2_t *iv )
{
  return(marker_map_pos_from_robot_pos(rpos,map,iv,REC));
}


logtools_vector2_t
google2vector( logtools_ll_coord_t ll, int zoom )
{
  logtools_vector2_t  v;

  int aa = 256 * (1 << (17 - zoom));
  double pixelsPerLonDegree = (double)aa / 360.0;
  double pixelsPerLonRadian = (double)aa / (2.0 * M_PI);
  double c = (double)aa / 2.0;
  double origX = c;
  double origY = c;
  double e = sin(ll.latitude * M_PI / 180.0);
  
  v.x = origX + ll.longitude * pixelsPerLonDegree;

  if (e > 0.9999)
    e = 0.9999;
  if (e < -0.9999)
    e = -0.9999;
  
  v.y = origY + 0.5 * log((1 + e) /
			  (1 - e)) * -pixelsPerLonRadian;
  
  return (v);
}

void
ImageMagickDrawPath( Image * image, ImageInfo * image_info,
		     logtools_grid_map2_t * map, logtools_log_data_t * rec, int upto )
{
  logtools_vector2_t           v, vec1, vec2, vstart={0,0}, vend;
  int               i, idx, plot = FALSE;
  DrawContext       wand;
#if MagickLibVersion >= 0x600
  PixelWand       * color;
#else
  DrawInfo          draw_info;
#endif
  double            size;
  logtools_rpos2_t             pos = {0.0, 0.0, 0.0};
  logtools_rmove2_t            nomove = {0.0, 0.0, 0.0};
  logtools_ll_coord_t          ll;
  logtools_ll_coord_t          llend = {0.0, 0.0};
  logtools_ll_coord_t          llstart = {0.0, 0.0};
  logtools_utm_coord_t         utmstart= {0.0, 0.0, 0, 'X'}, utmend= {0.0, 0.0, 0, 'X'}, utm= {0.0, 0.0, 0, 'X'};
  double            northingf, eastingf;
  
#if MagickLibVersion >= 0x600
  image_info = NULL;
  color = NewPixelWand();
  wand  = DrawAllocateWand( (DrawInfo*) NULL, image );
#else
  GetDrawInfo( image_info, &draw_info );
  wand = DrawAllocateContext ( &draw_info, image ); 
#endif
  DrawPushGraphicContext( wand );
  {
#if MagickLibVersion >= 0x600
    PixelSetColor( color, settings.pathcolor );
    DrawSetStrokeColor( wand, color );
    PixelSetColor( color, "none" );
    DrawSetFillColor( wand, color );
#else
    DrawSetStrokeColorString( wand, settings.pathcolor ); 
    DrawSetFillColorString( wand, "none" );
#endif
    
    if (1) {
    } else {
      DrawSetStrokeWidth( wand, settings.pathwidth );
      DrawPathStart( wand );
    }
    if (settings.utm_correct) {
      llstart.longitude = settings.posstart.x;
      llstart.latitude  = settings.posstart.y;
      utmstart = logtools_ll2utm( llstart );	
      llend.longitude = settings.posstart.x + settings.background.width  * settings.resolution_x;
      llend.latitude  = settings.posstart.y + settings.background.height * settings.resolution_y;
      utmend   = logtools_ll2utm( llend );
    }
    if (settings.google_correct) {
      llstart.longitude = settings.posstart.x;
      llstart.latitude  = settings.posstart.y;
      vstart = google2vector( llstart, settings.google_zoom );
      llend.longitude = settings.posstart.x +
	settings.background.width  * settings.resolution_x;
      llend.latitude  = settings.posstart.y +
	settings.background.height * settings.resolution_y;
      vend = google2vector( llend, settings.google_zoom );
    }
    for (i=0; i<upto; i++) {
      idx = rec->entry[i].index;
      if (settings.gpspath) {
	if (rec->entry[i].type==GPS) {
	  if (settings.utm_correct) {
	    ll.longitude = rec->gps[idx].longitude;
	    ll.latitude  = rec->gps[idx].latitude;
	    utm = logtools_ll2utm( ll );
	    northingf =
	      ( utm.northing - utmstart.northing ) / ( utmend.northing-utmstart.northing );
	    eastingf =
	      ( utm.easting - utmstart.easting ) / ( utmend.easting-utmstart.easting );
	    pos.x = llstart.longitude + eastingf * ( llend.longitude - llstart.longitude );
	    pos.y = llstart.latitude + northingf * ( llend.latitude - llstart.latitude );
	  } else if (settings.google_correct) {
	    ll.longitude = rec->gps[idx].longitude;
	    ll.latitude  = rec->gps[idx].latitude;
	    v = google2vector( ll, settings.google_zoom );
	    northingf = ( v.y - vstart.y ) / ( vend.y-vstart.y );
	    pos.y = llstart.latitude +
	      0.92 * northingf * ( llend.latitude - llstart.latitude );
	    // 0.96
	    pos.x = rec->gps[idx].longitude;
	  } else {
	    pos.x = rec->gps[idx].longitude;
	    pos.y = rec->gps[idx].latitude;
	  }
	  pos.o = 0.0;
	  plot = TRUE;
	}
      } else {
	if (rec->info.system==CARMEN) {
	  if (rec->entry[i].type==LASER_VALUES &&
	      rec->lsens[idx].id==settings.laser_id) {
	    pos = rec->lsens[idx].estpos;
	    plot = TRUE;
	  }
	} else if (rec->entry[i].type==POSITION) {
	  pos = rec->psens[idx].rpos; 
	  plot = TRUE;
	}
      }
      if (plot) {
	if (1) {
	  DrawPushGraphicContext( wand );
	  {
	    double   opacity       = 1.;
	    double   strokeopacity = 1.;
	    double   strokewidth   = 1.0;
	    char     stdcolor[MAX_CMD_LENGTH];
	    char     strokecolor[MAX_CMD_LENGTH];
	    strncpy( stdcolor,       "red",   MAX_CMD_LENGTH );
	    strncpy( strokecolor,    "none",   MAX_CMD_LENGTH );
	    PixelSetColor( color, strokecolor );
	    DrawSetStrokeColor( wand, color );
	    PixelSetColor( color, stdcolor );
	    DrawSetFillColor( wand, color );
	    DrawSetFillOpacity( wand, opacity );
	    DrawSetStrokeOpacity( wand, strokeopacity );
	    DrawSetStrokeWidth( wand, strokewidth );
	    magick_pos_without_rotation( pos, map, &vec1 );
	    DrawCircle( wand, vec1.x, vec1.y,
			vec1.x+settings.pathwidth,
			vec1.y+settings.pathwidth );
	  }
	  DrawPopGraphicContext( wand );
	} else {
	  magick_pos_without_rotation( pos, map, &vec1 );
	  //magick_pos_from_rpos( pos, map, &vec1 );
	  DrawPathLineToAbsolute ( wand, vec1.x, vec1.y );
	  plot = FALSE;
	}
      }
    }
    if (1) {
    } else {
      DrawPathFinish( wand );
    }
  }
  DrawPopGraphicContext( wand );
  if (settings.animation && !settings.gpspath) {
    DrawPushGraphicContext( wand );
    { 
      magick_pos_without_rotation( pos, map, &vec1 );
#if MagickLibVersion >= 0x600
      PixelSetColor( color, ROBOT_BORDER_COLOR );
      DrawSetStrokeColor( wand, color );
      PixelSetColor( color, ROBOT_FILL_COLOR );
      DrawSetFillColor( wand, color );
#else
      DrawSetStrokeColorString( wand, ROBOT_BORDER_COLOR ); 
      DrawSetFillColorString( wand, ROBOT_FILL_COLOR );
#endif
      DrawSetStrokeWidth( wand, 1.0 );
      size = metric_to_pixel( ROBOT_SIZE/2.0, map, REC );
      DrawCircle( wand, vec1.x, vec1.y, vec1.x+size, vec1.y+size );
      v = logtools_compute_laser_points( pos, ROBOT_SIZE/1.8, nomove, 0.0 );
      magick_pos_from_vector( v, map, &vec2 );
      DrawLine( wand, vec1.x, vec1.y, vec2.x, vec2.y );
    }
    DrawPopGraphicContext( wand );
  }
  DrawRender( wand );
#if MagickLibVersion >= 0x600
  DestroyDrawingWand( wand );
  DestroyPixelWand( color );
#else
  DrawDestroyContext( wand ); 
#endif
}

int
count_dumps_in_marker_line( char * data )
{
  int               ctr, dctr;
  char            * ptr, * running;
  char              str1[MAX_LINE_LENGTH];
  char              buffer1[MAX_LINE_LENGTH];
  char              buffer2[MAX_LINE_LENGTH];
  char              command[MAX_LINE_LENGTH];

  sscanf( data, "%s", str1 );
  if (!strncmp( str1, "[", MAX_LINE_LENGTH )) {
    sscanf( data, "[%[^]]", buffer1 );
  } else {
    strncpy( buffer1, data, MAX_LINE_LENGTH );
  }
  strncpy( buffer2, buffer1, MAX_LINE_LENGTH );
  running = buffer1;
  
  ctr = 0; dctr = 0;
  while ((ptr=strtok_r( ctr==0?running:NULL, ";",(char **) &buffer2))!=NULL) {
    sscanf( ptr, "%[^=]", str1 ); sscanf( str1, "%s", command );
    if (!strncasecmp( "dump", command, MAX_LINE_LENGTH)) {
      dctr++;
    }
    ctr++;
  }
  return(dctr);
}

int
ImageMagickDrawMapMarker( Image * image, ImageInfo * image_info,
			  logtools_grid_map2_t * map, char * data,
			  enum logtools_file_t system, int up_to_dump )
{
#if MagickLibVersion >= 0x600
  PixelWand       * color;
#else
  DrawInfo          draw_info;
#endif
  DrawContext       wand;
  
  logtools_vector2_t           vec, vec1, vec2, boxsize;
  logtools_value_set_t         dash_set = { 0, NULL };
  int               i, n, ctr, cctr, dctr, pri;
  logtools_rpos2_t             pos, pos1, pos2;
  char            * ptr, * running, * str;
  char              str1[MAX_LINE_LENGTH];
  char              str2[MAX_LINE_LENGTH];
  char              str3[MAX_LINE_LENGTH];
  char              str4[MAX_LINE_LENGTH];
  char              str5[MAX_LINE_LENGTH];
  char              buffer1[MAX_LINE_LENGTH];
  char              buffer2[MAX_LINE_LENGTH];
  char              dummy[MAX_LINE_LENGTH];
  char              command[MAX_LINE_LENGTH];

  char              stdcolor[MAX_CMD_LENGTH];
  char              fontname[MAX_CMD_LENGTH];
  char              strokecolor[MAX_CMD_LENGTH];
  char              strokepattern[MAX_CMD_LENGTH];
  char              textundercolor[MAX_CMD_LENGTH];
  
  double            circlesize    = 100.0;         /* in cm */
  double            fontsize      = 20.0;
  double            opacity       = 1.0;
  double            strokeopacity = 1.0;
  double            strokewidth   = 1.0;
  double            orientation   = 0; 
  double            size          = 0;
  double            val           = 0;
  int               error         = FALSE;

  PointInfo         rarrow[7];

#if MagickLibVersion >= 0x600
  image_info = NULL;
  color = NewPixelWand();
  wand  = DrawAllocateWand( (DrawInfo*) NULL, image );
#else
  GetDrawInfo( image_info, &draw_info );
  wand = DrawAllocateContext ( &draw_info, image ); 
#endif
  
  strncpy( stdcolor,       "blue",   MAX_CMD_LENGTH );
  strncpy( strokecolor,    "none",   MAX_CMD_LENGTH );
  strncpy( textundercolor, "white",  MAX_CMD_LENGTH );
  dash_set.numvalues = 0;

  sscanf( data, "%s", str1 );
  if (!strncmp( str1, "[", MAX_LINE_LENGTH )) {
    sscanf( data, "[%[^]]", buffer1 );
  } else {
    strncpy( buffer1, data, MAX_LINE_LENGTH );
  }
  running = buffer1;

  ctr = 0; dctr = 0; pri = 0;
  while ((ptr=strtok_r( ctr==0?running:NULL, ";",(char **) &buffer2))!=NULL) {
    sscanf( ptr, "%[^=]", str1 ); sscanf( str1, "%s", command );
    if (!strncasecmp( "dump", command, MAX_LINE_LENGTH)) {
      if (up_to_dump>0) {
	dctr++;
	if (dctr>=up_to_dump)
	  break;
      }
    } else if (!strncasecmp( "color", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	strncpy( stdcolor, str, MAX_CMD_LENGTH );
    } else if (!strncasecmp( "strokecolor", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	strncpy( strokecolor, str, MAX_CMD_LENGTH );
    } else if (!strncasecmp( "strokedash", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      sscanf( str1, " { %[^}] ", str2 );
      if (strlen(str2)>0) {	
	cctr=0;
	while ((ptr=strtok( cctr==0?str2:NULL, ","))!=NULL) {
	  cctr++;
	}
	if (dash_set.numvalues>0)
	  free(dash_set.val);
	dash_set.val = (double *) malloc( ( cctr + 1 ) * sizeof(double) );
	sscanf( str1, " { %[^}] ", str2 );
	cctr=0;
	while ((ptr=strtok( cctr==0?str2:NULL, ","))!=NULL) {
	  dash_set.val[cctr] = atof(ptr);
	  cctr++;
	}
	dash_set.val[cctr++] = 0.0;
	dash_set.numvalues = cctr;
      }
    } else if (!strncasecmp( "textundercolor", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	strncpy( textundercolor, str, MAX_CMD_LENGTH );
    } else if (!strncasecmp( "font", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	strncpy( fontname, str, MAX_CMD_LENGTH );
    } else if (!strncasecmp( "opacity", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	opacity = atof(str);
    } else if (!strncasecmp( "orientation", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	orientation = deg2rad(atof(str));
    } else if (!strncasecmp( "strokeopacity", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	strokeopacity = atof(str);
    } else if (!strncasecmp( "strokewidth", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	strokewidth = atof(str); 
    } else if (!strncasecmp( "strokepattern", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	strncpy( strokepattern, str, MAX_CMD_LENGTH );
    } else if (!strncasecmp( "fontsize", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr(str1);
      if (str!=NULL)
	fontsize = atof(str);
    } else if (!strncasecmp( "point", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      if (logtools_str_get_numbers( str1, 2, &(pos.x), &(pos.y) ) == 2 ) {
	marker_map_pos_from_robot_pos( pos, map, &vec, system );
	DrawPushGraphicContext( wand );
	{
	  DrawPoint( wand, vec.x, vec.y );
#if MagickLibVersion >= 0x600
	  PixelSetColor( color, strokecolor );
	  DrawSetStrokeColor( wand, color );
#else
	  DrawSetStrokeColorString( wand, strokecolor );
#endif
	  DrawSetStrokeOpacity( wand, strokeopacity );
	  DrawSetStrokeWidth( wand, strokewidth );
	  if (dash_set.numvalues>0)
	    DrawSetStrokeDashArray(wand, dash_set.numvalues-1, dash_set.val);
	  if (strcmp(strokepattern,"")) {
	    DrawSetStrokePatternURL( wand, strokepattern );
	  }
	}
	DrawPopGraphicContext( wand );
	pri++;
      }
    } else if (!strncasecmp( "rectangle", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      if (logtools_str_get_numbers( str1, 4, &(pos1.x), &(pos1.y),
			   &(pos2.x), &(pos2.y) ) == 4 ) {
	pos.x   = (pos2.x+pos1.x)/2.0;
	pos.y   = (pos2.y+pos1.y)/2.0;
	marker_map_pos_from_robot_pos( pos, map, &vec, system );
	boxsize.x  = metric_to_pixel( (pos2.x-pos1.x), map, system );
	boxsize.y  = metric_to_pixel( (pos2.y-pos1.y), map, system  );
	DrawPushGraphicContext( wand );
	{
	  DrawSetFillOpacity( wand, opacity );
#if MagickLibVersion >= 0x600
	  PixelSetColor( color, strokecolor );
	  DrawSetStrokeColor( wand, color );
	  PixelSetColor( color, stdcolor );
	  DrawSetFillColor( wand, color );
#else
	  DrawSetStrokeColorString( wand, strokecolor );
	  DrawSetFillColorString( wand, stdcolor );
#endif
	  DrawSetStrokeOpacity( wand, strokeopacity );
	  DrawSetStrokeWidth( wand, strokewidth );
	  if (dash_set.numvalues>0)
	    DrawSetStrokeDashArray(wand, dash_set.numvalues-1, dash_set.val);
	  if (strcmp(strokepattern,"")) {
	    DrawSetStrokePatternURL( wand, strokepattern );
	  }
	  if ( (fabs(orientation)>MIN_ROTATION) ||
	       (fabs(settings.rotation_angle)>MIN_ROTATION) ) {
	    DrawPushGraphicContext( wand );
	    {
	      DrawRotate(wand,rad2deg(-(orientation+settings.rotation_angle)));
	      DrawRectangle( wand, -boxsize.x/2.0, -boxsize.y/2.0,
			     boxsize.x/2.0, boxsize.y/2.0 );
	    }
	    DrawPopGraphicContext( wand );
	    pri++;
	  } else {
	    DrawRectangle( wand, -boxsize.x/2.0, -boxsize.y/2.0,
			   boxsize.x/2.0, boxsize.y/2.0 );
	  }
	  DrawTranslate( wand, vec.x, vec.y );
	}
	DrawPopGraphicContext( wand );
	pri++;
      }
    } else if (!strncasecmp( "line", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      if (logtools_str_get_numbers( str1, 4, &(pos1.x), &(pos1.y),
			   &(pos2.x), &(pos2.y) ) == 4 ) {
	marker_map_pos_from_robot_pos( pos1, map, &vec1, system );
	marker_map_pos_from_robot_pos( pos2, map, &vec2, system );
	DrawPushGraphicContext( wand );
	{
	  DrawSetFillOpacity( wand, opacity );
#if MagickLibVersion >= 0x600
	  PixelSetColor( color, strokecolor );
	  DrawSetStrokeColor( wand, color );
	  PixelSetColor( color, stdcolor );
	  DrawSetFillColor( wand, color );
#else
	  DrawSetStrokeColorString( wand, strokecolor );
	  DrawSetFillColorString( wand, stdcolor );
#endif
	  DrawSetStrokeOpacity( wand, strokeopacity );
	  DrawSetStrokeWidth( wand, strokewidth );
	  if (dash_set.numvalues>0)
	    DrawSetStrokeDashArray(wand, dash_set.numvalues-1, dash_set.val);
	  if (strcmp(strokepattern,"")) {
	    DrawSetStrokePatternURL( wand, strokepattern );
	  }
	}
	DrawLine( wand, vec1.x, vec1.y, vec2.x, vec2.y );
	DrawPopGraphicContext( wand );
	pri++;
      }
    } else if (!strncasecmp( "text", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      str = logtools_str_get_valstr( str1 );
      sscanf( str, "%[^,],%[^,],%[^:]", str2, str3, str4 );
      pos.x = atof( str2 );
      pos.y = atof( str3 );
      marker_map_pos_from_robot_pos( pos, map, &vec, system );
      if (str4[0]=='{') {
	sscanf( str4, "{%[^}]", str5 );
      } else {
	sscanf( str4, "%[^{]{%[^}]", dummy, str5 );
      }
      if ( strlen(str4) == strlen(dummy) ) {
	if (str4[0]=='\"') {
	  sscanf( str4, "\"%[^}]", str5 );
	} else {
	  sscanf( str4, "%[^\"]\"%[^\"]", dummy, str5 );
	}
	if ( strlen(str4) == strlen(str5) ) {
	  strncpy( str5, str4, MAX_LINE_LENGTH ); 
	}
      }
      DrawPushGraphicContext( wand );
      {
	if (strncmp( "none", strokecolor, MAX_LINE_LENGTH)) {
#if MagickLibVersion >= 0x600
	  PixelSetColor( color, strokecolor );
	  DrawSetStrokeColor( wand, color );
#else
	  DrawSetStrokeColorString( wand, strokecolor );
#endif
	  DrawSetStrokeOpacity( wand, strokeopacity );
	  DrawSetStrokeWidth( wand, strokewidth );
	  if (dash_set.numvalues>0)
	    DrawSetStrokeDashArray(wand, dash_set.numvalues-1, dash_set.val);
	  if (strcmp(strokepattern,"")) {
	    DrawSetStrokePatternURL( wand, strokepattern );
	  }
	}
	if (strcmp(fontname,"")) {
	  DrawSetFont( wand, fontname );
	} 
	DrawSetFillOpacity( wand, opacity );
#if MagickLibVersion >= 0x600
	PixelSetColor( color, textundercolor );
	DrawSetTextUnderColor( wand, color );
	PixelSetColor( color, stdcolor );
	DrawSetFillColor( wand, color );
#else
	DrawSetFillColorString( wand, stdcolor );
	DrawSetTextUnderColorString ( wand, textundercolor );
#endif
	DrawSetFontSize ( wand, fontsize );
	DrawTranslate( wand, vec.x, vec.y );
	if ( fabs(orientation)>MIN_ROTATION ) {
	  DrawPushGraphicContext( wand );
	  {
	    DrawRotate( wand,rad2deg(-orientation) );
	    DrawAnnotation ( wand, 0, (fontsize/2.5), (unsigned char*) str5 );
	  }
	  DrawPopGraphicContext( wand );
	  pri++;
	} else {
	  DrawAnnotation ( wand, 0, (fontsize/2.5),  (unsigned char*) str5 );
	}
      }
      DrawPopGraphicContext( wand );
      pri++;
  } else if (!strncasecmp( "circle", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      n = logtools_str_get_numbers( str1, 3, &(pos.x), &(pos.y), &val );
      if (n>=2) {
	if (n==3) {
	  size  = metric_to_pixel( val/(2.0*sqrt(2)), map, system );
	} else {
	  if (system==CARMEN)
	    circlesize/=100.0;
	  size  = metric_to_pixel( circlesize/(2.0*sqrt(2)), map, system );
	}
	marker_map_pos_from_robot_pos( pos, map, &vec, system );
	DrawPushGraphicContext( wand );
	{
#if MagickLibVersion >= 0x600
	  PixelSetColor( color, strokecolor );
	  DrawSetStrokeColor( wand, color );
	  PixelSetColor( color, stdcolor );
	  DrawSetFillColor( wand, color );
#else
	  DrawSetFillColorString( wand, stdcolor );
	  DrawSetStrokeColorString( wand, strokecolor );
#endif
	  DrawSetFillOpacity( wand, opacity );
	  DrawSetStrokeOpacity( wand, strokeopacity );
	  DrawSetStrokeWidth( wand, strokewidth );
	  if (dash_set.numvalues>0)
	    DrawSetStrokeDashArray(wand, dash_set.numvalues-1, dash_set.val);
	  if (strcmp(strokepattern,"")) {
	    DrawSetStrokePatternURL( wand, strokepattern );
	  }
	  DrawCircle( wand, vec.x, vec.y, vec.x+size, vec.y+size );
	}
	DrawPopGraphicContext( wand );
	pri++;
      }
    } else if (!strncasecmp( "path", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      sscanf( str1, " { %[^}] ", str2 );
      if (strlen(str2)>0) {
	DrawPushGraphicContext( wand );
	{
	  DrawSetFillOpacity( wand, opacity );
#if MagickLibVersion >= 0x600
	  PixelSetColor( color, strokecolor );
	  DrawSetStrokeColor( wand, color );
	  PixelSetColor( color, stdcolor );
	  DrawSetFillColor( wand, color );
#else
	  DrawSetFillColorString( wand, stdcolor );
	  DrawSetStrokeColorString( wand, strokecolor );
#endif
	  DrawSetStrokeOpacity( wand, strokeopacity );
	  DrawSetStrokeWidth( wand, strokewidth );
	  if (dash_set.numvalues>0)
	    DrawSetStrokeDashArray(wand, dash_set.numvalues-1, dash_set.val);
	  if (strcmp(strokepattern,"")) {
	    DrawSetStrokePatternURL( wand, strokepattern );
	  }
	  DrawPathStart( wand );
	  cctr=0;
	  while ((ptr=strtok( cctr==0?str2:NULL, ","))!=NULL) {
	    if (cctr%2==0) {
	      pos.x = atof( ptr );
	    } else {
	      pos.y = atof( ptr );
	      marker_map_pos_from_robot_pos( pos, map, &vec, system );
	      DrawPathLineToAbsolute ( wand, vec.x, vec.y );
	    }
	    cctr++;
	  }
	  DrawPathFinish( wand );
	}
	DrawPopGraphicContext( wand );
	pri++;
      }
    } else if (!strncasecmp( "arrow", command, MAX_LINE_LENGTH)) {
      sscanf( ptr, "%[^=]=%[^:]", dummy, str1 );
      n = logtools_str_get_numbers( str1, 3, &(pos.x), &(pos.y), &val );
      if (n>=2) {
	if (n==3) {
	  size  = metric_to_pixel( val/2.3, map, system );
	} else {
	  size  = metric_to_pixel( circlesize/2.3, map, system );
	}
	marker_map_pos_from_robot_pos( pos, map, &vec, system );
	DrawPushGraphicContext( wand );
	{
#if MagickLibVersion >= 0x600
	  PixelSetColor( color, strokecolor );
	  DrawSetStrokeColor( wand, color );
	  PixelSetColor( color, stdcolor );
	  DrawSetFillColor( wand, color );
#else
	  DrawSetFillColorString( wand, stdcolor );
	  DrawSetStrokeColorString( wand, strokecolor );
#endif
	  DrawSetStrokeWidth( wand, strokewidth );
	  if (dash_set.numvalues>0)
	    DrawSetStrokeDashArray(wand, dash_set.numvalues-1, dash_set.val);
	  if (strcmp(strokepattern,"")) {
	    DrawSetStrokePatternURL( wand, strokepattern );
	  }
	  for (i=0; i<7; i++) {
	    rarrow[i].x = size * arrow[i].x;
	    rarrow[i].y = size * arrow[i].y;
	  }
	  DrawTranslate( wand, vec.x, vec.y );
	  if ( (fabs(orientation)>MIN_ROTATION) ||
	       (fabs(settings.rotation_angle)>MIN_ROTATION) ) {
	    DrawPushGraphicContext( wand );
	    {
	      DrawRotate(wand,rad2deg(-(orientation+settings.rotation_angle)));
	      DrawPolygon( wand, 7, rarrow );
	    }
	    DrawPopGraphicContext( wand );
	    pri++;
	  } else {
	    DrawPolygon( wand, 7, rarrow );
	  }
	}
	DrawPopGraphicContext( wand );
	pri++;
      }
    } else {
      fprintf( stderr, "\n# WARNING: unknown marker command %s !", command );
      error = TRUE;
    }
    ctr++;
  }
  if (pri>0)
    DrawRender( wand );
#if MagickLibVersion >= 0x600
  DestroyDrawingWand( wand );
  DestroyPixelWand( color );
#else
  DrawDestroyContext( wand ); 
#endif
  return(!error);
}

void
check_active_markers( logtools_ivalue_set_t * active, logtools_log_data_t * rec, int up_to_entry )
{
  char   dummy[MAX_STRING_LENGTH];
  char   tagstr[MAX_STRING_LENGTH];
  int    i, j, idx;
  for (i=0;i<up_to_entry;i++) {
    if (rec->entry[i].type==MARKER) {
      idx = rec->entry[i].index;
      if (idx>=0 && idx<active->numvalues && active->val[idx]) {
	if (sscanf( rec->marker[idx].datastr,
		    "[ remove %[^]]", dummy )==1) {
	  sscanf( dummy, "%s", tagstr );
	  /* REMOVE marker will be deactivated */
	  active->val[idx]=FALSE;
	  for (j=0; j<idx; j++) {
	    if (!strcmp( rec->marker[j].tag, tagstr )) {
	      active->val[j]=FALSE;
	    }
	  }
	}
      }
    }
  }
}

void
copy_probs_to_data( log2pic_image_t * img, logtools_grid_map2_t * map,
		    logtools_ivector2_t zsize )
{
  logtools_gauss_kernel_t      kernel;
  int                          i, j, idx, ix, iy;
  double                       c;
  
  if (settings.endpoints) {
    log2pic_map_compute_probs( map, 0.0 );
  } else {
    log2pic_map_compute_probs( map, settings.unknown_val );
  }
  if (settings.convolve) {
    kernel = logtools_compute_gauss_kernel( settings.kernel_size );
    log2pic_simple_convolve_map( map, kernel );
  }
  for (i=0;i<zsize.x;i++) {
    for (j=0;j<zsize.y;j++) {
      idx = j*zsize.x+i;
      ix = (int) (i/map->zoom);
      iy = (int) (j/map->zoom);
      if (ix>=0 && iy>=0 && ix<map->mapsize.x && iy<map->mapsize.y) {
	if (map->mapsum[ix][map->mapsize.y-iy-1]==0) {
	  if (settings.bgfile) {
	    if (ix>=0 && iy>=0 &&
		ix<settings.background.width &&
		iy<settings.background.height) {
	      img->pixel[idx*4]     = 1.0; 
	      img->pixel[idx*4+1]   = settings.background.pixel[ix][iy].r;
	      img->pixel[idx*4+2]   = settings.background.pixel[ix][iy].g;
	      img->pixel[idx*4+3]   = settings.background.pixel[ix][iy].b;
	    } else {
	      img->pixel[idx*4]     = 1.0; 
	      img->pixel[idx*4+1]   = settings.bg.r; 
	      img->pixel[idx*4+2]   = settings.bg.g;
	      img->pixel[idx*4+3]   = settings.bg.b;
	    }
	  } else if (settings.set_size) {
	    img->pixel[idx*4]     = 0.0; 
	    img->pixel[idx*4+1]   = 1.0;
	    img->pixel[idx*4+2]   = 1.0;
	    img->pixel[idx*4+3]   = 1.0; 
	  } else {
	    img->pixel[idx*4]     = 1.0; 
	    img->pixel[idx*4+1]   = settings.bg.r; 
	    img->pixel[idx*4+2]   = settings.bg.g;
	    img->pixel[idx*4+3]   = settings.bg.b;
	  }
	} else {
	  c = 1.0-map->mapprob[ix][map->mapsize.y-iy-1];
	  if (c<0.0)
	    c = 0.0;
	  img->pixel[idx*4]     = 1.0; 
	  img->pixel[idx*4+1]   = c;
	  img->pixel[idx*4+2]   = c;
	  img->pixel[idx*4+3]   = c;
	}
      }
    }
  }
}

void
alloc_image_from_data( Image ** image, log2pic_image_t * img, logtools_grid_map2_t * map )
{
  ExceptionInfo       exception;
  GetExceptionInfo( &exception );
  *image = ConstituteImage ( map->zoom * map->mapsize.x,
			     map->zoom * map->mapsize.y, "ARGB",
			     DoublePixel, img->pixel, &exception );
  if (*image == (Image *) NULL) {
    fprintf( stderr, "ERROR: no memory!!!\n" );
    exit(1);
  }
}

void
draw_image_marker( Image * image, ImageInfo * image_info,
		   logtools_grid_map2_t * map, logtools_log_data_t * rec,
		   int up_to_entry, int up_to_dump )
{
  static logtools_ivalue_set_t   active;
  static int          firsttime = TRUE;
  int                 i, idx;
  char                markerstr[MAX_LINE_LENGTH];
  if (firsttime) {
    active.val = (int *) malloc( rec->nummarkers * sizeof(int) );
    active.numvalues = rec->nummarkers;
    for (i=0;i<active.numvalues;i++) {
      active.val[i] = TRUE;
    }
    firsttime = FALSE;
  }
  check_active_markers( &active, rec, up_to_entry );
  for (i=0; i<up_to_entry; i++) {
    idx = rec->entry[i].index;
    if (rec->entry[i].type==MARKER) {
      if (active.val[idx]) {
	sscanf( rec->marker[idx].datastr, "[%[^]]", markerstr );
	if (strlen(markerstr)>0) {
	  if (!ImageMagickDrawMapMarker( image, image_info, map,
					 markerstr, rec->info.system,
					 up_to_dump )) {
	    fprintf( stderr, "\n# WARNING: error in line %d !\n# ... ",
		     rec->entry[i].linenr+1 );
	  }
	}
      }
    }
  }
  if (settings.showpath) {
    fprintf( stderr, "draw path ... " );
#if MagickLibVersion >= 0x600
    ImageMagickDrawPath( image, image_info, map, rec, up_to_entry ); 
#else
    ImageMagickDrawPath( image, image_info, map, rec, up_to_entry ); 
#endif
  }
}

void
log2pic_dump_animation_map( Image * image, ImageInfo * image_info,
			    log2pic_image_t * img, logtools_grid_map2_t * map,
			    logtools_ivector2_t zsize, logtools_log_data_t * rec,
			    int up_to_entry, int up_to_scan, int up_to_dump )
{
  double progress;
  copy_probs_to_data( img, map, zsize );
  alloc_image_from_data( &image, img, map );
  if (settings.gpspath) {
    ImageMagickDrawPath( image, image_info, map, rec, up_to_entry ); 
  } else {
    draw_image_marker( image, image_info, map, rec, up_to_entry, up_to_dump );
  }
  strcpy( (image->filename), log2pic_dump_filename() );
  fprintf( stderr, "# write image in file %s ... ",
	   image->filename );
  WriteImage( image_info, image );
  if (settings.gpspath) {
    fprintf( stderr, "done\n" );
  } else {
    progress = 100.0 * (up_to_scan-settings.from) /
      (double) ( ( settings.to<rec->numlaserscans?
		   settings.to:rec->numlaserscans ) - settings.from );
    if (progress>100.0)
      progress = 100.0;
    fprintf( stderr, "done [scan: %d - %3.2f%%]\n", up_to_scan, progress );
  }
  DestroyImage(image);
}

void
log2pic_write_image_magick_map( logtools_grid_map2_t * map, logtools_log_data_t * rec )
{
  int                 ok = TRUE;
  int                 i, j, num_dumps, idx, size, ctr = 0, lidx = 0;
  Image             * image = NULL;
  ImageInfo           image_info;
  logtools_ivector2_t            zsize;
  ExceptionInfo       exception;
  PixelPacket         color;
  log2pic_image_t     img;
  logtools_rpos2_t               lastpos = {MAXFLOAT,MAXFLOAT,0};
  zsize.x = (int) (map->zoom * map->mapsize.x);
  zsize.y = (int) (map->zoom * map->mapsize.y);
  GetExceptionInfo(&exception);
  QueryColorDatabase( settings.bgcolor, &color, &exception);
  settings.bg.r = color.red / (double) MaxRGB;
  settings.bg.g = color.green / (double) MaxRGB;
  settings.bg.b = color.blue / (double) MaxRGB;
  size = zsize.x * zsize.y * 4 * sizeof(double);
  fprintf( stderr, "# alloc memory of pixel map (%.1f MB) ... ",
	   (size/(1024.0*1024.0)) );
  if ( (img.pixel = (double *) malloc(size))==NULL )
    ok = FALSE;
  fprintf( stderr, "%s\n", ok?"yes":"no" );
  GetImageInfo(&image_info);
  
  if (settings.animation) {
    /*************  ANIMATION **************/
    log2pic_filetemplate_from_filename( settings.filetemplate,
					settings.outfilename );
    if (settings.gpspath) {
      for (i=0; i<rec->numentries; i++) {
	idx = rec->entry[i].index;
	if (rec->entry[i].type==GPS) {
	  if (ctr++%(settings.anim_skip+1)==0) {
	    log2pic_dump_animation_map( image, &image_info, &img,
					map, zsize, rec, i, idx+1, 0 );
	  }
	}
      }
    } else {
      if (settings.from==0)
	log2pic_dump_animation_map( image, &image_info, &img,
				    map, zsize, rec,
				    0, 0, 0 );
      lidx = 0;
      for (i=0; i<rec->numentries; i++) {
	idx = rec->entry[i].index;
	if (rec->entry[i].type==LASER_VALUES) {
	  if (rec->lsens[idx].id==settings.laser_id) {
	    lidx = idx+1;
	    log2pic_map_integrate_scan( map, rec->lsens[idx],
					settings.max_range,
					settings.usable_range );
	    if (logtools_rpos2_distance(lastpos,
				rec->lsens[idx].estpos)>settings.anim_step) {
	      if ((settings.from < 0|| idx>=settings.from) && (settings.to < 0|| idx<=settings.to))
		if (ctr++%(settings.anim_skip+1)==0) {
		  log2pic_dump_animation_map( image, &image_info, &img,
					      map, zsize, rec, i, idx+1, 0 );
		}
	      lastpos = rec->lsens[idx].estpos;
	    }
	  }
	} else if (rec->entry[i].type==MARKER) {
	  num_dumps = count_dumps_in_marker_line( rec->marker[idx].datastr );
	  for (j=0; j<num_dumps; j++) {
	    if (ctr++%(settings.anim_skip+1)==0) {
	      log2pic_dump_animation_map( image, &image_info, &img,
					  map, zsize, rec, i, lidx, j+1 );
	    }
	  }
	}
      }
      if ((settings.from < 0|| rec->numlaserscans>settings.from) &&
	  (settings.to < 0|| rec->numlaserscans<settings.to))
	log2pic_dump_animation_map( image, &image_info, &img, map, zsize,
				    rec, rec->numentries, rec->numlaserscans,
				    0 );
    }
  } else {
    /*************    IMAGE   **************/
    fprintf( stderr, "# integrate laser data ... " );
    for (i=0; i<rec->numentries; i++) {
      idx = rec->entry[i].index;
      if (rec->entry[i].type==LASER_VALUES) {
        if (rec->lsens[idx].id==settings.laser_id) {
	  log2pic_map_integrate_scan( map, rec->lsens[idx],
				      settings.max_range,
				      settings.usable_range );
	}
	lidx = idx;
      } else if (rec->entry[i].type==MARKER) {
	num_dumps = count_dumps_in_marker_line( rec->marker[idx].datastr );
	for (j=0; j<num_dumps; j++) {
	  if (ctr++%(settings.anim_skip+1)==0) {
	    log2pic_filetemplate_from_filename( settings.filetemplate,
						settings.outfilename );
	    log2pic_dump_animation_map( image, &image_info, &img,
					map, zsize, rec, i, lidx, j+1 );
	  }
	}
      }
    }
    copy_probs_to_data( &img, map, zsize );
    fprintf( stderr, "done\n" );
    fprintf( stderr, "# create image of map ... " );
    alloc_image_from_data( &image, &img, map );
    fprintf( stderr, "done\n" );
    fprintf( stderr, "# draw marker ... " );
    draw_image_marker( image, &image_info, map, rec, rec->numentries, 0 );
    fprintf( stderr, "done\n" );
    strcpy( (image->filename), settings.outfilename );
    fprintf( stderr, "# write image in file %s ... ", image->filename );
    WriteImage( &image_info, image );
    fprintf( stderr, "done\n" );
    DestroyImage(image);
  }
  DestroyConstitute();
  free(img.pixel);
}

void
log2pic_read_image_file( char * filename, log2pic_background_image_t * img )
{
  ExceptionInfo                  exception;
  Image                        * image;
  ImageInfo                    * image_info;
  register const PixelPacket   * pixels;
  int                            idx, x, y;

  GetExceptionInfo( &exception );
  image_info= CloneImageInfo( (ImageInfo *) NULL );
  strcpy( image_info->filename, filename );
  image = ReadImage( image_info, &exception );
  if (exception.severity != UndefinedException)
    CatchException(&exception);
  if (image == (Image *) NULL) {
    fprintf( stderr, "# ERROR: can read image file %s !\n",
	     filename );
    exit(1);
  }
  fprintf( stderr, "# INFO: read image file %s (%dx%d)\n",
	   filename, (int) image->columns, (int) image->rows );
  img->start.x = 0.0;
  img->start.y = 0.0;
  img->width   = image->columns; 
  img->height  = image->rows;
  fprintf( stderr, "# INFO: allocating memory ... " );
  img->pixel = (RGB **) mdalloc( 2, sizeof(RGB),
				 img->width, img->height );
  fprintf( stderr, "done\n" );
  pixels = GetImagePixels( image, 0, 0, img->width, img->height );
  for (x=0; x<img->width; x++) {
    for (y=0; y<img->height; y++) {
      idx = y*img->width+x;
      img->pixel[x][y].r  = pixels[idx].red   / 65536.0;
      img->pixel[x][y].g  = pixels[idx].green / 65536.0;
      img->pixel[x][y].b  = pixels[idx].blue  / 65536.0;
    }
  }
  DestroyImage(image);
  DestroyImageInfo(image_info);
  DestroyExceptionInfo(&exception);
  
}

