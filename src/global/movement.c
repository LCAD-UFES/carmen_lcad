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

#include "global.h"
#include "movement.h"


/* ** Rotation-Translation-Rotation Transformations *********** */

carmen_point_t carmen_movement_rtr_move_point(carmen_point_t pt, 
			      carmen_movement_rtr_t move) {
  pt.theta += move.r1;
  pt.x += move.d * cos(pt.theta);
  pt.y += move.d * sin(pt.theta);
  pt.theta = carmen_normalize_theta(pt.theta + move.r2);
  return pt;
}

carmen_movement_rtr_t  carmen_movement_compose_rtr(carmen_movement_rtr_t move1,
				    carmen_movement_rtr_t move2) {

  carmen_point_t pt_zero = {0.0, 0.0, 0.0};
  carmen_point_t pt;
  pt =  carmen_movement_rtr_move_point(pt_zero, move1);
  pt =  carmen_movement_rtr_move_point(pt, move2);
  return carmen_movement_rtr_between_points(pt_zero, pt);
}

carmen_movement_rtr_t carmen_movement_rtr_between_points(carmen_point_t pt1, 
					  carmen_point_t pt2) {
  carmen_movement_rtr_t rel;
  rel.r1 = carmen_normalize_theta(atan2(pt2.y - pt1.y, 
					pt2.x - pt1.x) - pt1.theta);
  rel.d  = hypot(pt2.x - pt1.x, pt2.y - pt1.y);
  rel.r2 = carmen_normalize_theta(pt2.theta - pt1.theta - rel.r1);
  return rel;
}

carmen_movement_rtr_t carmen_movement_rtr_from_point(carmen_point_t pt) {
  carmen_point_t pt_zero = {0.0, 0.0, 0.0};
  return  carmen_movement_rtr_between_points(pt_zero, pt);
}

carmen_point_t carmen_movement_point_from_rtr(carmen_movement_rtr_t move) {
  carmen_point_t pt_zero = {0.0, 0.0, 0.0};
  return  carmen_movement_rtr_move_point(pt_zero, move);
}

carmen_movement_rtr_t carmen_movement_invert_rtr(carmen_movement_rtr_t r) {

  carmen_movement_rtr_t t = r;
  t.r1 = carmen_normalize_theta(-M_PI-r.r2);
  t.r2 = carmen_normalize_theta(-M_PI-r.r1);
  return t;
}


/* ** Forward-Sideward-Rotation Transformations *********** */

carmen_point_t carmen_movement_fsr_move_point(carmen_point_t pt, 
			      carmen_movement_fsr_t move) {
  carmen_point_t pt2 = pt;
  pt2.x += move.f * cos(pt.theta) - move.s * sin(pt.theta);
  pt2.y += move.f * sin(pt.theta) + move.s * cos(pt.theta);
  pt2.theta = carmen_normalize_theta(move.r + pt.theta);
  return pt2;
}

carmen_movement_fsr_t  carmen_movement_compose_fsr(carmen_movement_fsr_t move1,
				    carmen_movement_fsr_t move2) {
  carmen_movement_fsr_t comp;
  comp.f = cos(move1.r) * move2.f - sin(move1.r) * move2.s + move1.f;
  comp.s = sin(move1.r) * move2.f + cos(move1.r) * move2.s + move1.s;
  comp.r = carmen_normalize_theta(move1.r  + move2.r);
  return comp;
}

carmen_movement_fsr_t carmen_movement_fsr_between_points(carmen_point_t pt1, 
							 carmen_point_t pt2) {
  carmen_movement_fsr_t move;

  move.f =   (pt2.y - pt1.y) * sin(pt1.theta) + (pt2.x - pt1.x) * cos(pt1.theta);
  move.s = + (pt2.y - pt1.y) * cos(pt1.theta) - (pt2.x - pt1.x) * sin(pt1.theta);
  move.r = carmen_normalize_theta( pt2.theta - pt1.theta );
  return move;
}

carmen_movement_fsr_t carmen_movement_fsr_from_point(carmen_point_t pt) {
  carmen_point_t pt_zero = {0.0, 0.0, 0.0};
  return carmen_movement_fsr_between_points(pt_zero, pt);
}

carmen_point_t carmen_movement_point_from_fsr(carmen_movement_fsr_t move) {
  carmen_point_t pt_zero = {0.0, 0.0, 0.0};
  return  carmen_movement_fsr_move_point(pt_zero, move);
}

carmen_movement_fsr_t carmen_movement_invert_fsr(carmen_movement_fsr_t move) {

  carmen_movement_fsr_t p_inv;
  p_inv.f = - cos(move.r) * move.f - sin(move.r) * move.s;
  p_inv.s =   sin(move.r) * move.f - cos(move.r) * move.s;
  p_inv.r = carmen_normalize_theta(-move.r);
  return p_inv;
}

/* ** Coversion between FSR and RTR Transformations *********** */

carmen_movement_rtr_t carmen_movement_fsr_to_rtr(carmen_movement_fsr_t move) {
  return carmen_movement_rtr_from_point(carmen_movement_point_from_fsr(move));
}

carmen_movement_fsr_t carmen_movement_rtr_to_fsr(carmen_movement_rtr_t move) {
  return carmen_movement_fsr_from_point(carmen_movement_point_from_rtr(move));
}

/* ** Transformation between coordinate frames ************ */

/* returns pt_frame1 in the coordinate frame2 */
carmen_point_t carmen_movement_transformation_between_frames(carmen_point_t reference_pt_frame1, 
							     carmen_point_t reference_pt_frame2,
							     carmen_point_t pt_frame1) {

  carmen_movement_rtr_t itrans_refp1 = 
    carmen_movement_invert_rtr( carmen_movement_rtr_from_point(reference_pt_frame1) );

  carmen_movement_rtr_t trans_refp2 = 
    carmen_movement_rtr_from_point(reference_pt_frame2);

  carmen_movement_rtr_t trans_pt = 
    carmen_movement_rtr_from_point(pt_frame1);

  return 
    carmen_movement_point_from_rtr(  
				   carmen_movement_compose_rtr(  
							       carmen_movement_compose_rtr( trans_refp2,
											    itrans_refp1) ,
							       trans_pt )
				     );
}


