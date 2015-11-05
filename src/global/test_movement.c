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
#include "global.h"
#include "movement.h"


int is_close_pt(carmen_point_t p1, carmen_point_t p2) {

  fprintf(stderr, "test (%.2f %.2f %.3f) == (%.2f %.2f %.3f)\t\t",
	  p1.x, p1.y, p1.theta,  p2.x, p2.y, p2.theta );
  
  if ( hypot(p1.x-p2.x, p1.y-p2.y) < 0.0001 && 
       fabs(carmen_normalize_theta(p1.theta-p2.theta)) < 0.01*M_PI) {
    fprintf(stderr, "OK\n");
    return 1;
  }
  fprintf(stderr, "ERROR\n");  
  return 0;
}

int is_close_fsr(carmen_movement_fsr_t m1, carmen_movement_fsr_t m2) {

  carmen_point_t t1 = {0.0, 0.0, 0.0};
  carmen_point_t t2 = carmen_movement_fsr_move_point(t1, m1);
  carmen_point_t t3 = carmen_movement_fsr_move_point(t1, m2);

  return is_close_pt(t2, t3);
}

int is_close_rtr(carmen_movement_rtr_t m1, carmen_movement_rtr_t m2) {

  carmen_point_t t1 = {0.0, 0.0, 0.0};
  carmen_point_t t2 = carmen_movement_rtr_move_point(t1, m1);
  carmen_point_t t3 = carmen_movement_rtr_move_point(t1, m2);

  return is_close_pt(t2, t3);
}


int is_close_rtr_fsr(carmen_movement_rtr_t m1, carmen_movement_fsr_t m2) {

  carmen_point_t t1 = {0.0, 0.0, 0.0};
  carmen_point_t t2 = carmen_movement_rtr_move_point(t1, m1);
  carmen_point_t t3 = carmen_movement_fsr_move_point(t1, m2);

  return is_close_pt(t2, t3);
}

int main(int argc, char** argv) {

  if (argc != 1)
    carmen_die("Syntax: %s\n", argv[0]);

  carmen_generate_random_seed();

  carmen_point_t t1 = { carmen_uniform_random(-10.0, 10.0), 
			carmen_uniform_random(-10.0, 10.0), 
			carmen_uniform_random(-M_PI, M_PI)};
  carmen_point_t t3,t4;

  carmen_movement_rtr_t rtr1 =  { carmen_uniform_random(-10.0, 10.0), 
			      carmen_uniform_random(-10.0, 10.0), 
			      carmen_uniform_random(-M_PI, M_PI)};
  carmen_movement_fsr_t fsr1 = carmen_movement_rtr_to_fsr(rtr1); 
  //test
  is_close_rtr_fsr(rtr1, fsr1);

  //test
  is_close_rtr_fsr(carmen_movement_fsr_to_rtr(fsr1), fsr1);

  //test
  is_close_rtr_fsr(rtr1, carmen_movement_rtr_to_fsr(rtr1));

  //test
  is_close_rtr(rtr1, carmen_movement_invert_rtr(carmen_movement_invert_rtr(rtr1)) );

  //test
  is_close_fsr(fsr1, carmen_movement_invert_fsr(carmen_movement_invert_fsr(fsr1)) );
  
  //test
  is_close_rtr_fsr(carmen_movement_invert_rtr(rtr1), carmen_movement_invert_fsr(fsr1));
  
  t3 = carmen_movement_rtr_move_point(t1, rtr1);
  carmen_movement_rtr_t rtr2 = carmen_movement_rtr_between_points(t1, t3);
  //test
  is_close_rtr(rtr1, rtr2);

  t3 = carmen_movement_fsr_move_point(t1, fsr1);
  carmen_movement_fsr_t fsr2 = carmen_movement_fsr_between_points(t1, t3);
  //test
  is_close_fsr(fsr1, fsr2);
 
  //test
  is_close_rtr_fsr(rtr2, fsr2);

  rtr2 = carmen_movement_invert_rtr(rtr2);
  fsr2 = carmen_movement_invert_fsr(fsr2);

  is_close_rtr_fsr(rtr2, fsr2);

  t3 = carmen_movement_rtr_move_point(t1, rtr2);
  t4 = carmen_movement_fsr_move_point(t1, fsr2);

  is_close_pt(t3, t4);

  carmen_point_t tb = { carmen_uniform_random(-10.0, 10.0), 
			carmen_uniform_random(-10.0, 10.0), 
			carmen_uniform_random(-M_PI, M_PI)};

  carmen_point_t tc = { carmen_uniform_random(-10.0, 10.0), 
			carmen_uniform_random(-10.0, 10.0), 
			carmen_uniform_random(-M_PI, M_PI)};

  carmen_point_t ta = { carmen_uniform_random(-10.0, 10.0), 
			carmen_uniform_random(-10.0, 10.0), 
			carmen_uniform_random(-M_PI, M_PI)};


  carmen_point_t td = carmen_movement_transformation_between_frames(ta, tb, tc);
  is_close_pt(tc,  carmen_movement_transformation_between_frames(tb, ta, td) );


  is_close_pt(td,  carmen_movement_transformation_between_frames(ta, ta, td) );

  
  

 
  return 0;
}
