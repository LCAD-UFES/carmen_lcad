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

/** @addtogroup maptools liblinemapping **/
// @{

/** 
 * \file linemapping.h 
 * \brief Library for generating linemaps
 *
 * Split'n'Merge based library for generating linemaps fram range data.
 **/

#ifndef CARMEN_LINEMAPPING_H
#define CARMEN_LINEMAPPING_H

#ifdef __cplusplus
extern "C" {
#endif

#define CARMEN_LINEMAPPING_ROBOT_FRAME  1
#define CARMEN_LINEMAPPING_GLOBAL_FRAME 0

typedef struct{
  carmen_point_t p1, p2;
  int weight;
} carmen_linemapping_segment_t;


typedef struct{
  int num_segs;
  carmen_linemapping_segment_t *segs;
} carmen_linemapping_segment_set_t;


typedef struct{
  double laser_max_length;
  double laser_opening_angle;
  double laser_start_angle;
  double sam_tolerance;
  double sam_max_gap;
  double sam_min_length;
  int    sam_min_num;
  int    sam_use_fit_split; 
  double merge_max_dist;
  double merge_min_relative_overlap;
  double merge_overlap_min_length;
  double merge_uniformly_distribute_dist;
} carmen_linemapping_parameters_t;


void carmen_linemapping_init(int argc, char** argv);


carmen_linemapping_segment_set_t  
carmen_linemapping_get_segments_from_scan(const carmen_robot_laser_message *scan, 
					  int local);

carmen_linemapping_segment_set_t 
carmen_linemapping_get_segments_from_beams(const carmen_robot_laser_message *scan, 
					   int local, 
					   int from, int to);

carmen_linemapping_segment_set_t  
carmen_linemapping_get_segments_from_scans(const carmen_robot_laser_message *multiple_scans, 
					   int num_scans);

void                            
carmen_linemapping_update_linemap(carmen_linemapping_segment_set_t *linemap, 
				  const carmen_robot_laser_message *laser);

void 
carmen_linemapping_free_segments(carmen_linemapping_segment_set_t* s);


/* some utility functions for linesegments */

double 
carmen_linemapping_angle_difference(const carmen_linemapping_segment_t *s1, 
				    const carmen_linemapping_segment_t *s2);

double          
carmen_linemapping_distance_point_point(const carmen_point_t *p1, 
					const carmen_point_t *p2);

double          
carmen_linemapping_segment_length(const carmen_linemapping_segment_t *s);


double 
carmen_linemapping_distance_point_linesegment(const carmen_linemapping_segment_t *l, 
					      const carmen_point_t *p);

double 
carmen_linemapping_distance_linesegment_linesegment(const carmen_linemapping_segment_t *l1,
						    const carmen_linemapping_segment_t *l2);



#ifdef __cplusplus
}
#endif


#endif 


// @}
