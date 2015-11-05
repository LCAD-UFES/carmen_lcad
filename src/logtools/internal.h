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

#include <carmen/logtools.h>

int           rec2_parse_line( char *line, logtools_log_data_t *rec, int alloc, int verbose );

int           carmen_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode );

int           moos_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode );

int           player_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode );

int           saphira_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode );

int           placelab_parse_line( char *line, logtools_log_data_t *rec, int alloc, int mode );

int           load_rec2d_file( char *filename, logtools_log_data_t *rec, enum logtools_file_t type, int mode );

int           read_script( char *filename, logtools_log_data_t *script, int verbose );


/****************************************************************************/

double                   convert_orientation_to_range( double angle );

double                   compute_orientation_diff( double start, double end );

void                     robot2map( logtools_rpos2_t pos, logtools_rpos2_t corr, logtools_rpos2_t *map );

void                     map2robot( logtools_rpos2_t map, logtools_rpos2_t corr, logtools_rpos2_t *pos );

void                     computeCorr( logtools_rpos2_t pos, logtools_rpos2_t map, logtools_rpos2_t *corr );

void                     compute_forward_correction( logtools_rpos2_t pos, logtools_rpos2_t corr, logtools_rpos2_t *cpos );

void                     compute_backward_correction( logtools_rpos2_t cpos, logtools_rpos2_t corr, logtools_rpos2_t *pos );

void                     compute_correction_parameters( logtools_rpos2_t pos, logtools_rpos2_t cpos, logtools_rpos2_t *corr );

void                     update_correction_parameters( logtools_rpos2_t cpos, logtools_rpos2_t delta, logtools_rpos2_t *corr );

/****************************************************************************/

double        double_time( struct timeval time );

void          convert_time( double tval, struct timeval *time );

int           timeCompare ( struct timeval time1, struct timeval time2 );

double        timeDiff( struct timeval  t1, struct timeval t2);

double        get_user_time( void );

/****************************************************************************/

double        sick_laser_fov( int numbeams );

double        sick_laser_angle_diff( int num, double fov );

char *        extended_filename( char * filename );

char *        printable_filename( char * filename );

FILE *        fopen_filename( char * filename, char * mode );

double        gps_degree_abs_decimal( double degree_minute );

double        gps_degree_decimal( double degree_minute, char orient );



