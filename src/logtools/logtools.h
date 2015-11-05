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

#ifndef LOGTOOLS_UTILS_H
#define LOGTOOLS_UTILS_H

#ifndef _STDIO_H
#include <stdio.h>
#endif

#ifndef	_MATH_H
#include <math.h>
#endif

#ifndef _SYS_TIME_H
#include <sys/time.h>
#endif

#define rad2deg rad2Deg

#ifndef rad2deg
#define rad2deg rad2Deg
#endif

#ifndef deg2rad
#define deg2rad deg2Rad
#endif

#ifndef MIN
#define MIN(x,y) (x < y ? x : y)
#endif

#ifndef MIN3
#define MIN3(x,y,z) MIN(MIN(x,y),z)
#endif

#ifndef MAX
#define MAX(x,y) (x > y ? x : y)
#endif

#ifndef MAX3
#define MAX3(x,y,z) MAX(MAX(x,y),z)
#endif

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef M_TWO_PI
#define M_TWO_PI  6.2831853071795864769252867665590058
#endif


typedef struct {

  double                         x;
  double                         y;

} logtools_vector2_t;

typedef struct {

  int                            numvectors;
  logtools_vector2_t           * vec;

} logtools_vector2_t_SET;

typedef struct {

  int                            x;
  int                            y;

} logtools_ivector2_t;

typedef struct {

  int                            numvectors;
  logtools_ivector2_t          * vec;

} logtools_ivector2_set_t;

typedef struct {

  double                         x;
  double                         y;
  double                         o;

} logtools_rpos2_t;

typedef struct {

  double                         forward;
  double                         sideward;
  double                         rotation;

} logtools_rmove2_t;

typedef struct {

  double                         tv;
  double                         rv;

} logtools_rvel2_t;

typedef struct {

  double                         easting;
  double                         northing;
  int                            zone;
  char                           letter;

} logtools_utm_coord_t;

typedef struct {

  double                         longitude;
  double                         latitude;

} logtools_ll_coord_t;

typedef struct {

  struct timeval                 time;
  double                         utc;
  double                         latitude;
  char                           lat_orient;
  double                         longitude;
  char                           long_orient;
  int                            gps_quality;
  int                            num_satellites;
  int                            status;
  double                         hdop;
  double                         sea_level;
  double                         altitude;
  double                         geo_sea_level;
  double                         geo_sep; 
  double                         speed_over_ground;
  int                            data_age;

} logtools_gps_data_t;

typedef struct {

  logtools_vector2_t             min;
  logtools_vector2_t             max;

} logtools_bounding_box_t;

typedef struct {

  logtools_vector2_t             relpt;
  logtools_vector2_t             abspt;
  int                            tag;
  int                            info;

} logtools_laser_coord2_t;

typedef struct {

  double                         start;
  double                         end;
  double                         delta;

} logtools_laser_fov_t;

typedef struct {

  logtools_rmove2_t              offset;
  logtools_laser_fov_t           fov;

} logtools_laser_prop2_t;

typedef struct {

  struct timeval                 time;
  int                            partial;
  int                            numvalues;
  double                         fov;
  logtools_rmove2_t              offset;
  float                        * val;
  float                        * angle;

} logtools_laser_data_t;

typedef struct {

  logtools_rpos2_t               estpos;
  logtools_laser_data_t          laser;
  logtools_laser_coord2_t      * coord;
  logtools_bounding_box_t        bbox;
  int                            id;

} logtools_lasersens2_data_t;

typedef struct {

  struct timeval                 time;
  logtools_rpos2_t               rpos;
  logtools_rvel2_t               rvel;

} logtools_possens2_data_t;

typedef struct {

  struct timeval                 time;
  char                         * mac;
  char                         * essid;
  char                         * protocol;
  char                         * mode;
  int                            nwid;       
  int                            channel;    
  int                            encryption; 
  int                            rate;       
  int                            beacon;     
  int                            signal;
  int                            link;
  int                            noise;
  char                           precloc;
  float                          distance;

} logtools_wifi_data_t;

enum logtools_file_t    { SCRIPT,
			  REC,
			  CARMEN,
			  MOOS,
			  PLAYER,
			  SAPHIRA,
			  PLACELAB,
			  UNKOWN };

enum logtools_entry_t   { POSITION,
			  LASER_VALUES,
			  GPS,
			  WIFI,
			  MARKER,
			  UNKNOWN };


typedef struct {

  enum logtools_entry_t    type;
  int                      index;
  int                      idx1, idx2;
  int                      linenr;

} logtools_entry_position_t;

typedef struct {

  enum logtools_file_t     system;

} logtools_rec2_info_t;


typedef struct {
  
  struct timeval      time;
  char              * datastr; 
  char              * tag;

} logtools_marker_data_t;

typedef struct {

  /* entries */
  int                            numentries;
  logtools_entry_position_t    * entry;  

  /* positions */
  int                            numpositions;
  logtools_possens2_data_t     * psens;
  
  /* laser scans */
  int                            numlaserscans;
  logtools_lasersens2_data_t   * lsens;

  int                            numwifi;
  logtools_wifi_data_t         * wifi;

  int                            numgps;
  logtools_gps_data_t          * gps;

  int                            nummarkers;
  logtools_marker_data_t      *  marker;
  
  logtools_rec2_info_t           info;
  
} logtools_log_data_t;

typedef struct {
  
  int                            numvalues;
  double                       * val;

} logtools_value_set_t;

typedef struct {

  int                            numvalues;
  int                          * val;

} logtools_ivalue_set_t;

typedef struct {

  int                            len;
  double                       * val;

} logtools_gauss_kernel_t;

typedef struct {

  int                            numgrids;
  logtools_ivector2_t          * grid;

} logtools_grid_line_t;

typedef struct {

  logtools_rpos2_t               offset;
  double                         resolution;
  float                       ** maphit;
  short                       ** mapsum;
  float                       ** mapprob;
  float                       ** calc;
  logtools_ivector2_t            mapsize;
  logtools_vector2_t             center;
  double                         zoom;

} logtools_grid_map2_t;


/****************************************************************************/
/*                                                                          */
/*                                                                          */
/*                                                                          */
/****************************************************************************/

double        rad2Deg(double x);

double        deg2Rad(double x);

double        fsgn( double val );

int           sgn( int val );

double        random_gauss( void );

void *        mdalloc(int ndim, int width, ...);

void          mdfree(void *tip, int ndim);

/****************************************************************************/

int                      logtools_read_logfile( logtools_log_data_t * rec,
						char * filename );

int                      logtools_write_logfile( logtools_log_data_t * rec,
						 char * filename );

/****************************************************************************/

logtools_vector2_t       logtools_rotate_vector2( logtools_vector2_t p, double angle );

logtools_vector2_t       logtools_logtools_rotate_and_translate_vector2( logtools_vector2_t p, double rot, logtools_vector2_t trans );

double                   logtools_vector2_length( logtools_vector2_t v1 );

double                   logtools_vector2_distance( logtools_vector2_t p1, logtools_vector2_t p2 );

double                   logtools_rpos2_distance( logtools_rpos2_t p1, logtools_rpos2_t p2 );

/****************************************************************************/

logtools_rmove2_t        logtools_movement2_between_rpos2( logtools_rpos2_t s, logtools_rpos2_t e );

logtools_rpos2_t         logtools_rpos2_with_movement2( logtools_rpos2_t start, logtools_rmove2_t move );

logtools_rpos2_t         logtools_rpos2_backwards_with_movement2( logtools_rpos2_t start, logtools_rmove2_t move );

/****************************************************************************/

void                     logtools_compute_coordpts( logtools_log_data_t *rec );

logtools_vector2_t       logtools_compute_laser_points( logtools_rpos2_t rpos, double val, logtools_rmove2_t offset, double angle );

/****************************************************************************/

double                   logtools_gauss_function( double x, double mu, double sigma );

logtools_gauss_kernel_t  logtools_compute_gauss_kernel( int length );

/****************************************************************************/

logtools_utm_coord_t     logtools_ll2utm( logtools_ll_coord_t ll );

logtools_ll_coord_t      logtools_utm2ll( logtools_utm_coord_t utm );

/****************************************************************************/

char *                   logtools_str_get_valstr( char *str );

char *                   logtools_str_get_str( char *str );

int                      logtools_str_get_numbers( char *str, int num, ... );


#endif 

