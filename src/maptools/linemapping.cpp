

#include "linemapping.h"

#include "dynamictable.h"


typedef DynamicTable<carmen_point_t> carmen_linemapping_dyn_tab_point_t;
typedef DynamicTable<carmen_linemapping_segment_t> carmen_linemapping_dyn_tab_segment_t;

// global Variable of all needed parameters
carmen_linemapping_parameters_t carmen_linemapping_params_global;
const double carmen_linemapping_epsilon = 0.001;


int                             
carmen_linemapping_merge_segment(carmen_linemapping_segment_set_t *set, 
				 carmen_linemapping_segment_t *segment);

int                             
carmen_linemapping_merge_segment(carmen_linemapping_segment_set_t *set, 
				 carmen_linemapping_segment_t *segment, 
				 int index_no_element);

void                             
carmen_linemapping_line_fitting_uniformly_distribute(carmen_linemapping_segment_t *s1, 
						     const carmen_linemapping_segment_t *s2);

carmen_linemapping_dyn_tab_point_t*          
carmen_linemapping_uniformly_distribute_points_on_segment(const carmen_linemapping_segment_t *s1, 
							  const carmen_linemapping_segment_t *s2);


// general functions

int            
carmen_linemapping_overlap_point_linesegment(const carmen_linemapping_segment_t *s, 
					     const carmen_point_t *p);

int            
carmen_linemapping_intersection_line_line(const carmen_linemapping_segment_t *s1, 
					  const carmen_linemapping_segment_t *s2, 
					  carmen_point_t *cut_p);

carmen_point_t  
carmen_linemapping_get_point_on_segment(const carmen_linemapping_segment_t *s, 
					const carmen_linemapping_segment_t *s_over);





void
carmen_linemapping_get_params( int argc, char **argv, 
			       carmen_linemapping_parameters_t* param )
{
  carmen_param_t param_list[] = {
    
    {"linemapping", "laser_maxrange", CARMEN_PARAM_DOUBLE, 
     &param->laser_max_length, 0, NULL},
    {"linemapping", "sam_tolerance", CARMEN_PARAM_DOUBLE, 
     &param->sam_tolerance, 0, NULL},
    {"linemapping", "sam_max_gap", CARMEN_PARAM_DOUBLE, 
     &param->sam_max_gap, 0, NULL},
    {"linemapping", "sam_min_length", CARMEN_PARAM_DOUBLE, 
     &param->sam_min_length, 0, NULL},
    {"linemapping", "sam_min_num", CARMEN_PARAM_INT, 
     &param->sam_min_num, 0, NULL},
    {"linemapping", "sam_use_fit_split", CARMEN_PARAM_ONOFF, 
     &param->sam_use_fit_split, 0, NULL},

    {"linemapping", "merge_max_dist", CARMEN_PARAM_DOUBLE, 
     &param->merge_max_dist, 0, NULL},
    {"linemapping", "merge_min_relative_overlap", CARMEN_PARAM_DOUBLE, 
     &param->merge_min_relative_overlap, 0, NULL},
    {"linemapping", "merge_overlap_min_length", CARMEN_PARAM_DOUBLE, 
     &param->merge_overlap_min_length, 0, NULL},
    {"linemapping", "merge_uniformly_distribute_dist", CARMEN_PARAM_DOUBLE, 
     &param->merge_uniformly_distribute_dist, 0, NULL}
    
  };

  carmen_param_install_params(argc, argv, param_list, 
                              sizeof(param_list) / sizeof(param_list[0]));
                              
}



// initialize all needed parameters for the line mapping approach
void carmen_linemapping_init(int argc, char** argv)
{
  carmen_linemapping_get_params(argc, argv, &carmen_linemapping_params_global);
}

// returns the x-coordinate on the line segment 's', given the y-coordinate
double carmen_linemapping_get_x_coordinate_on_segment(const carmen_linemapping_segment_t *s, 
						      double y)
{
  return ( (y - s->p1.y) / (s->p2.y - s->p1.y) )*(s->p2.x - s->p1.x) + s->p1.x;
}

// returns the y-coordinate on the line segment 's', given the x-coordinate
double carmen_linemapping_get_y_coordinate_on_segment(const carmen_linemapping_segment_t *s, 
						      double x)
{
  return ( (x - s->p1.x) / (s->p2.x - s->p1.x) )*(s->p2.y - s->p1.y) + s->p1.y;
}


// free the allocated space of a 'carmen_linemapping_segment_set_t'
void carmen_linemapping_free_segments(carmen_linemapping_segment_set_t* s) {
  if (s->segs != NULL)
    free(s->segs);
  s->segs = NULL;
  s->num_segs = 0;
}

// calculates the  max. distance 'max_dist' of a point 'pnt[max_index]' of the point set 'pnts[i_low] ... pnts[i_high]'
// to the line segment with the endpoints 'p1' and 'p2'
void carmen_linemapping_get_max_dist_max_index(const carmen_point_t *p1, 
					       const carmen_point_t *p2, 
					       const carmen_linemapping_dyn_tab_point_t *pnts, 
					       int i_low, int i_high, double *max_dist, int *max_index)
{
  double dx = p2->x - p1->x;
  double dy = p2->y - p1->y;
  double denominator = carmen_square(dx) + carmen_square(dy);
  *max_dist = 0.0;
  if(denominator < carmen_square(carmen_linemapping_epsilon)){ // line segment is a point
    for(int i=i_low; i<=i_high; i++){
      double distance = carmen_linemapping_distance_point_point(p1, pnts->getElement(i));
      if(distance>*max_dist){ *max_dist = distance; *max_index = i; }
    }
  }
  else{
    for(int i=i_low; i<=i_high; i++){
      double numerator = carmen_square(dx*(pnts->getElement(i)->y - p2->y) - dy*(pnts->getElement(i)->x - p2->x));
      double distance = sqrt( numerator/denominator );
      if(distance>*max_dist){ *max_dist = distance; *max_index = i; }
    }
  }
}


// fits the line segment 's' to the point set 'pnts[i_low] ... pnts[i_high]'
void carmen_linemapping_line_fitting(carmen_linemapping_segment_t *s, 
				     const carmen_linemapping_dyn_tab_point_t *pnts, 
				     int i_low, int i_high)
{
  int num_of_points = i_high - i_low + 1;

  double x_av = 0.0, y_av = 0.0;
  for(int i=i_low; i<=i_high; i++){
    x_av += pnts->getElement(i)->x;
    y_av += pnts->getElement(i)->y;
  }
  x_av = x_av/(double)num_of_points;
  y_av = y_av/(double)num_of_points;

  double numerator = 0.0, denominator = 0.0;
  double min_x = MAXDOUBLE, max_x = -MAXDOUBLE, min_y = MAXDOUBLE, max_y = -MAXDOUBLE;
  for(int i=i_low; i<=i_high; i++){
    numerator   += (y_av - pnts->getElement(i)->y) * (x_av - pnts->getElement(i)->x);
    denominator += carmen_square( (y_av - pnts->getElement(i)->y) ) 
      - carmen_square( (x_av - pnts->getElement(i)->x)  );
    if( min_x > pnts->getElement(i)->x ){ min_x = pnts->getElement(i)->x; }
    if( max_x < pnts->getElement(i)->x ){ max_x = pnts->getElement(i)->x; }
    if( min_y > pnts->getElement(i)->y ){ min_y = pnts->getElement(i)->y; }
    if( max_y < pnts->getElement(i)->y ){ max_y = pnts->getElement(i)->y; }
  }
  numerator *= -2.0;

  double angle    = atan2( numerator, denominator ) / 2.0;
  double distance = ( x_av*cos(angle) ) + ( y_av*sin(angle) );
  if(angle<M_PI/4.0 && angle>-M_PI/4.0){
    s->p1.x = ( distance - (min_y)*sin(angle) ) / cos(angle);
    s->p2.x = ( distance - (max_y)*sin(angle) ) / cos(angle);
    s->p1.y = min_y;
    s->p2.y = max_y;
  }
  else{
    s->p1.y = ( distance - (min_x)*cos(angle) ) / sin(angle);
    s->p2.y = ( distance - (max_x)*cos(angle) ) / sin(angle);
    s->p1.x = min_x;
    s->p2.x = max_x;
  }

}



// recursiv "split and merge"-function
// - if the funktion is not called again recursivly then a new line segment will be created
// - a new line segment is created from the point set 'pnts[i_low] ... pnts[i_high]'
void carmen_linemapping_split_and_merge(carmen_linemapping_dyn_tab_segment_t *segs, 
					const carmen_linemapping_dyn_tab_point_t *pnts, 
					int i_low, int i_high)
{
  int num_of_points = i_high - i_low + 1;

  // check number of points
  if( num_of_points < carmen_linemapping_params_global.sam_min_num ){ return; }
  
  // check the length of the line segment
  double length = carmen_linemapping_distance_point_point(pnts->getElement(i_low), pnts->getElement(i_high));
  if( length < carmen_linemapping_params_global.sam_min_length )
    return; 
  
  // check the distance of neighboured points
  for(int i=i_low; i<i_high; i++){
    double dis = carmen_linemapping_distance_point_point(pnts->getElement(i), pnts->getElement(i+1));
    if( dis > carmen_linemapping_params_global.sam_max_gap ){
      if(i  !=i_low) 
	carmen_linemapping_split_and_merge(segs, pnts, i_low, i     );
      if(i+1!=i_high)
	carmen_linemapping_split_and_merge(segs, pnts, i+1  , i_high); 
      return;
    }
  }

  carmen_linemapping_segment_t *s = new carmen_linemapping_segment_t();
  carmen_test_alloc(s);

  // check the distance of the points to the line segment
  if( num_of_points > 2 ){
    int max_index=0;
    double max_dist = carmen_linemapping_params_global.sam_tolerance + 1.0;
    if(carmen_linemapping_params_global.sam_use_fit_split){ // line-fitting before 'split'-check
      carmen_linemapping_line_fitting(s, pnts, i_low, i_high);
      carmen_linemapping_get_max_dist_max_index(&s->p1, &s->p2, pnts, i_low, i_high, &max_dist, &max_index);
    }

    if( max_dist > carmen_linemapping_params_global.sam_tolerance ){
      carmen_linemapping_get_max_dist_max_index(pnts->getElement(i_low), pnts->getElement(i_high), pnts, i_low, i_high, &max_dist, &max_index);

      if(max_dist > carmen_linemapping_params_global.sam_tolerance || carmen_linemapping_params_global.sam_use_fit_split){
        delete s;
        carmen_linemapping_split_and_merge(segs, pnts, i_low,     max_index); // recursion
        carmen_linemapping_split_and_merge(segs, pnts, max_index, i_high   ); // recursion
        return;
      }
    }
  }


  if(!carmen_linemapping_params_global.sam_use_fit_split){   // line-fitting after 'split'-check
    carmen_linemapping_line_fitting(s, pnts, i_low, i_high); 
  } 

  s->p1.theta = i_low;
  s->p2.theta = i_high;
  if (atan2( s->p1.y, s->p1.x) > atan2( s->p2.y, s->p2.x) ) {
    carmen_point_t tmp;
    tmp = s->p2;
    s->p2 = s->p1;
    s->p1 = tmp;
  }
  s->weight = num_of_points;
  segs->add(s);

}




carmen_linemapping_segment_set_t 
carmen_linemapping_get_segments_from_points(const carmen_linemapping_dyn_tab_point_t *pnts) {

  carmen_linemapping_dyn_tab_segment_t *segments_temp = new carmen_linemapping_dyn_tab_segment_t(100);
  carmen_test_alloc(segments_temp);
  carmen_linemapping_split_and_merge(segments_temp, pnts, 0, pnts->numberOfElements()-1);
  
  // just copy the elements of 'segments_temp' into 'segments'
  carmen_linemapping_segment_set_t segments;
  segments.num_segs = segments_temp->numberOfElements();
  segments.segs = new carmen_linemapping_segment_t[segments.num_segs];
  carmen_test_alloc(segments.segs);
  carmen_linemapping_segment_t** elements = segments_temp->getElements();
  for(int i=0; i<segments.num_segs; i++){ segments.segs[i] = *(elements[i]); }

  segments_temp->setAutoDelete(true);
  delete segments_temp;
  return segments;
}


carmen_linemapping_dyn_tab_point_t* 
carmen_linemapping_get_points_local(const carmen_robot_laser_message *m, 
				    int from, int to)
{
  if (from < 0)
    from = 0;
  if (to > m->num_readings-1)
    to = m->num_readings-1;


  carmen_linemapping_dyn_tab_point_t *points = new carmen_linemapping_dyn_tab_point_t(to-from+1);
  carmen_test_alloc(points);

  for(int i=from; i<=to; i++){
    if( m->range[i] < carmen_linemapping_params_global.laser_max_length ){
      carmen_point_t *p = new carmen_point_t;
      carmen_test_alloc(p);
      double angle = m->config.start_angle + (i*m->config.angular_resolution);
      p->x = m->range[i] * cos(angle);
      p->y = m->range[i] * sin(angle);
      points->add(p);
    }
  }
  return points;

}


// calculates the global cartesian coordinates of the range data
carmen_linemapping_dyn_tab_point_t* 
carmen_linemapping_get_points_global(const carmen_robot_laser_message *m, 
				     int from, int to)
{
  if (from < 0)
    from = 0;
  if (to > m->num_readings-1)
    to = m->num_readings-1;

  carmen_linemapping_dyn_tab_point_t *points = new carmen_linemapping_dyn_tab_point_t(to-from+1);
  carmen_test_alloc(points);

  for(int i=from; i<=to; i++){
    if( m->range[i] < carmen_linemapping_params_global.laser_max_length ){
      carmen_point_t *p = new carmen_point_t;
      carmen_test_alloc(p);
      double angle = m->config.start_angle + m->laser_pose.theta + (i*m->config.angular_resolution);
      p->x = m->laser_pose.x + m->range[i] * cos(angle);
      p->y = m->laser_pose.y + m->range[i] * sin(angle);
      points->add(p);
    }
  }
  return points;

}


// calculates a set of line segments, given a set of range date
// this approach based on 'split and merge' aka 'interativ end-point fit'
// m: range data; robot position
// local: "true"  -> line segments are local; origin is the robot position
//        "false" -> line segments are global
// fromm,to: from beam index from to beam index to (inclusive)
carmen_linemapping_segment_set_t 
carmen_linemapping_get_segments_from_beams(const carmen_robot_laser_message *scan, 
					   int local, int from, int to)
{
  if( scan->num_readings <= 0){
    carmen_die("ERROR: reading carmen_robot_laser_message; num_readings <= 0\nin %s line %d\n",__FILE__,__LINE__);
  }

  carmen_linemapping_dyn_tab_point_t *pnts;
  if(local){ pnts = carmen_linemapping_get_points_local(scan, from, to); }
  else     { pnts = carmen_linemapping_get_points_global(scan, from, to); }

  carmen_linemapping_segment_set_t segs =   
    carmen_linemapping_get_segments_from_points(pnts);


  pnts->setAutoDelete(true);
  delete pnts;

  return segs;
}

carmen_linemapping_segment_set_t 
carmen_linemapping_get_segments_from_scan(const carmen_robot_laser_message *scan, 
					  int local) {
  return carmen_linemapping_get_segments_from_beams( scan, local, 0, scan->num_readings-1);
}


// calculates a set of line segments, given a set of scans
// this approach based on 'split and merge' aka 'interativ end-point fit'
// m: set of scans
// num_scans: number of scans
carmen_linemapping_segment_set_t 
carmen_linemapping_get_segments_from_scans(const carmen_robot_laser_message *multiple_scans, int num_scans)
{
  carmen_linemapping_segment_set_t segments 
    = carmen_linemapping_get_segments_from_scan(&multiple_scans[0], false);
  for(int i=1; i<num_scans; i++)
    carmen_linemapping_update_linemap(&segments, &multiple_scans[i]); 
  
  return segments;
}


// merges the two sets of line segments 'segments_old' and 'segments_new'
carmen_linemapping_segment_set_t* 
carmen_linemapping_merge_segment_sets(carmen_linemapping_segment_set_t *segments_old, 
				      carmen_linemapping_segment_set_t *segments_new)
{
  for(int i=0; i<segments_new->num_segs; i++){
    if(segments_new->segs[i].weight==0){ continue; }

    int merge_old = true, merge_new = false;
    while ( merge_old || merge_new ){ // merge untill nothing change
      merge_old = carmen_linemapping_merge_segment(segments_old, &segments_new->segs[i]);
      if(merge_old || merge_new){
        merge_new = carmen_linemapping_merge_segment(segments_new, &segments_new->segs[i], i);
      }
    }
  }

  int count = 0;
  for(int i_old=0; i_old<segments_old->num_segs; i_old++)
    if(segments_old->segs[i_old].weight!=0)
      count++; 
  for(int i_new=0; i_new<segments_new->num_segs; i_new++)
    if(segments_new->segs[i_new].weight!=0)
      count++;

  carmen_linemapping_segment_set_t *merge_segments = new carmen_linemapping_segment_set_t;
  carmen_test_alloc(merge_segments);
  merge_segments->num_segs = count;
  merge_segments->segs = new carmen_linemapping_segment_t[merge_segments->num_segs];
  carmen_test_alloc(merge_segments->segs);

  int index = 0;
  for(int i_old=0; i_old<segments_old->num_segs; i_old++){
    if(segments_old->segs[i_old].weight!=0){
      merge_segments->segs[index] = segments_old->segs[i_old];
      index++;
    }
  }

  for(int i_new=0; i_new<segments_new->num_segs; i_new++){
    if(segments_new->segs[i_new].weight!=0){
      merge_segments->segs[index] = segments_new->segs[i_new];
      index++;
    }
  }

  return merge_segments;

}


// updates a set of line segments 'linemap', given a new scan 'laser'
void carmen_linemapping_update_linemap(carmen_linemapping_segment_set_t *linemap, 
				       const carmen_robot_laser_message *laser)
{
  carmen_linemapping_segment_set_t laser_segments 
    = carmen_linemapping_get_segments_from_scan(laser, false);
  carmen_linemapping_segment_set_t *new_linemap   
    = carmen_linemapping_merge_segment_sets(linemap, &laser_segments);

  delete [] laser_segments.segs;

  if(linemap->num_segs>0){ delete [] linemap->segs; }
  linemap->num_segs = new_linemap->num_segs;
  linemap->segs = new_linemap->segs;

}


// calls: 'carmen_linemapping_merge_segmet(set, segment, -1)'
int carmen_linemapping_merge_segment(carmen_linemapping_segment_set_t *set, 
				     carmen_linemapping_segment_t *segment)
{
  return carmen_linemapping_merge_segment(set, segment, -1);

}


// - merges 'segment' with the best candidate of 'set', but not the candidate 'set[index_no_element]'
// - merged segment is stored in 'segment', weight of best canditate is set to '0'
int carmen_linemapping_merge_segment(carmen_linemapping_segment_set_t *set, 
				     carmen_linemapping_segment_t *segment, 
				     int index_no_element)
{
  int best_merge_index = -1;
  double best_similar_angle = MAXDOUBLE;;
  for(int i=0; i<set->num_segs; i++){
    if(i==index_no_element || set->segs[i].weight==0 || segment->weight==0){ continue; }

    carmen_linemapping_segment_t *s_set = &(set->segs[i]);

    double dx = segment->p2.x - segment->p1.x;
    double dy = segment->p2.y - segment->p1.y;
    double denominator = carmen_square(dx) + carmen_square(dy);

    double numerator1 = carmen_square( dx*(s_set->p1.y - segment->p2.y) - dy*(s_set->p1.x - segment->p2.x) );
    double numerator2 = carmen_square( dx*(s_set->p2.y - segment->p2.y) - dy*(s_set->p2.x - segment->p2.x) );
    double dis_set_p1 = sqrt( numerator1/denominator );
    double dis_set_p2 = sqrt( numerator2/denominator );

    if( dis_set_p1 < carmen_linemapping_params_global.merge_max_dist && 
	dis_set_p2 < carmen_linemapping_params_global.merge_max_dist ) { 
      // 's_set' is close enough to the line 'segment' (not line segment!!!)
      int overlap_set_p1 = carmen_linemapping_overlap_point_linesegment(segment, &s_set->p1);
      int overlap_set_p2 = carmen_linemapping_overlap_point_linesegment(segment, &s_set->p2);
      
      if( overlap_set_p1 && overlap_set_p2 ){ 
	// case 1: both endpoints of 's_set' overlaps -> merge
	double a_dif = carmen_linemapping_angle_difference(segment, s_set);
	if( a_dif < best_similar_angle ){
	  best_similar_angle = a_dif;
	  best_merge_index = i;
	  continue;
	}
      }
      
      if( overlap_set_p1 || overlap_set_p2 ){ 
	// case 2: only one endpoint of 's_set' overlaps -> test merge
	double overlap_dist;
	carmen_point_t point = carmen_linemapping_get_point_on_segment(segment, s_set);
	if( overlap_set_p1 ){ 
	  overlap_dist = carmen_linemapping_distance_point_point(&s_set->p1, &point); 
	}
	else                { 
	  overlap_dist = carmen_linemapping_distance_point_point(&s_set->p2, &point); 
	}
	
	double line_size = carmen_linemapping_distance_point_point(&s_set->p1, &s_set->p2);
	if( (overlap_dist/line_size) > carmen_linemapping_params_global.merge_min_relative_overlap || 
	    overlap_dist > carmen_linemapping_params_global.merge_overlap_min_length ){
	  double a_dif = carmen_linemapping_angle_difference(segment, s_set);
	  if( a_dif < best_similar_angle ){
	    best_similar_angle = a_dif;
	    best_merge_index = i;
	    continue;
	  }
	}
      }
    }
    
    dx = s_set->p2.x - s_set->p1.x;
    dy = s_set->p2.y - s_set->p1.y;
    denominator = carmen_square(dx) + carmen_square(dy);

    numerator1 = carmen_square( dx*(segment->p1.y - s_set->p2.y) - dy*(segment->p1.x - s_set->p2.x) );
    numerator2 = carmen_square( dx*(segment->p2.y - s_set->p2.y) - dy*(segment->p2.x - s_set->p2.x) );
    double dis_segment_p1 = sqrt( numerator1/denominator );
    double dis_segment_p2 = sqrt( numerator2/denominator );

    if( dis_segment_p1 < carmen_linemapping_params_global.merge_max_dist && 
	dis_segment_p2 < carmen_linemapping_params_global.merge_max_dist ){ 
      // 'segment' is close enough to the line 's_set' (not line segment!!!)
      int overlap_segment_p1 = carmen_linemapping_overlap_point_linesegment(s_set, &segment->p1);
      int overlap_segment_p2 = carmen_linemapping_overlap_point_linesegment(s_set, &segment->p2);

      if(overlap_segment_p1 && overlap_segment_p2){ // case 3: both endpoints of 's_old' overlaps -> merge
	double a_dif = carmen_linemapping_angle_difference(segment, s_set);
	if( a_dif < best_similar_angle ){
	  best_similar_angle = a_dif;
	  best_merge_index = i;
	  continue;
	}
      }

      if(overlap_segment_p1 || overlap_segment_p2){ // case 4: only one endpoint of 'segment' overlaps -> test merge
	double overlap_dist;
	carmen_point_t point = carmen_linemapping_get_point_on_segment(s_set, segment);
	if( overlap_segment_p1 ){ 
	  overlap_dist = carmen_linemapping_distance_point_point(&segment->p1, &point); 
	}
	else                    { 
	  overlap_dist = carmen_linemapping_distance_point_point(&segment->p2, &point); 
	}

	double line_size = carmen_linemapping_distance_point_point(&segment->p1, &segment->p2);
	if( (overlap_dist/line_size) > carmen_linemapping_params_global.merge_min_relative_overlap || 
	    overlap_dist > carmen_linemapping_params_global.merge_overlap_min_length ){
	  double a_dif = carmen_linemapping_angle_difference(segment, s_set);
	  if( a_dif < best_similar_angle ){
	    best_similar_angle = a_dif;
	    best_merge_index = i;
	    continue;
	  }
	}
      }
    }
  }
  
  if (best_merge_index>=0){ 
    // merge the best candidate with 'segment'
    carmen_linemapping_line_fitting_uniformly_distribute(segment, &(set->segs[best_merge_index]));
    set->segs[best_merge_index].weight = 0;
    return true;
  }
  else
    return false; 

}

// - merges 's1' with 's2' with a line fitting method
// - points are uniformly distributed on the line segments
// - number and weight of the points are calculated from the weight of the line segment
void carmen_linemapping_line_fitting_uniformly_distribute(carmen_linemapping_segment_t *s1, 
							  const carmen_linemapping_segment_t *s2)
{
  // get uniformly distributed point set
  carmen_linemapping_dyn_tab_point_t *points;
  points = carmen_linemapping_uniformly_distribute_points_on_segment(s1, s2);

  double x_av = 0.0, y_av = 0.0;
  for(int i=0; i<points->numberOfElements(); i++){
    carmen_point_t *p = points->getElement(i);
    x_av += p->x * p->theta;
    y_av += p->y * p->theta;
  }
  int weight_of_points = s1->weight + s2->weight;
  x_av = x_av / (double)weight_of_points;
  y_av = y_av / (double)weight_of_points;

  double numerator = 0.0, denominator = 0.0;
  for(int i=0; i<points->numberOfElements(); i++){
    carmen_point_t *p = points->getElement(i);
    numerator   += p->theta * (y_av - p->y) * (x_av - p->x);
    denominator += p->theta * ( carmen_square( (y_av - p->y) ) - carmen_square( (x_av - p->x) ) );
  }
  numerator *= -2.0;
  double angle    = atan2( numerator, denominator ) / 2.0;
  double distance = ( x_av*cos(angle) ) + ( y_av*sin(angle) );

  double min_x = MAXDOUBLE, max_x = -MAXDOUBLE, min_y = MAXDOUBLE, max_y = -MAXDOUBLE;
  const carmen_point_t* pnts[4] = {&s1->p1, &s1->p2, &s2->p1, &s2->p2};
  for(int i=0; i<=3; i++){
    const carmen_point_t *p = pnts[i];
    if( min_x > p->x ){ min_x = p->x; } if( max_x < p->x ){ max_x = p->x; }
    if( min_y > p->y ){ min_y = p->y; } if( max_y < p->y ){ max_y = p->y; }
  }

  if(angle<M_PI/4.0 && angle>-M_PI/4.0){
    s1->p1.x = ( distance - (min_y)*sin(angle) ) / cos(angle);
    s1->p2.x = ( distance - (max_y)*sin(angle) ) / cos(angle);
    s1->p1.y = min_y;
    s1->p2.y = max_y;
  }
  else{
    s1->p1.y = ( distance - (min_x)*cos(angle) ) / sin(angle);
    s1->p2.y = ( distance - (max_x)*cos(angle) ) / sin(angle);
    s1->p1.x = min_x;
    s1->p2.x = max_x;
  }
  s1->weight = weight_of_points;

  points->setAutoDelete(true);
  delete points;

}

// - uniformly distribute points on each of the two line segments 's1' and 's2'
// - returns the set of these points
carmen_linemapping_dyn_tab_point_t* 
carmen_linemapping_uniformly_distribute_points_on_segment(const carmen_linemapping_segment_t *s1, 
							  const carmen_linemapping_segment_t *s2)
{
  carmen_linemapping_dyn_tab_point_t *points = new carmen_linemapping_dyn_tab_point_t(100);
  carmen_test_alloc(points);

  const carmen_linemapping_segment_t *segments[2] = {s1, s2};
  for(int i=0; i<=1; i++){
    const carmen_linemapping_segment_t *s = segments[i];
    double dx = fabs(s->p1.x - s->p2.x);
    double dy = fabs(s->p1.y - s->p2.y);
    if( dx > dy ){
      double start_x, end_x;
      if( s->p1.x < s->p2.x ){ start_x = s->p1.x; end_x = s->p2.x; }
      else                   { start_x = s->p2.x; end_x = s->p1.x; }

      int num_discrete_points = (int)( dx / carmen_linemapping_params_global.merge_uniformly_distribute_dist );
      double tmp = dx - ((double)num_discrete_points)*carmen_linemapping_params_global.merge_uniformly_distribute_dist;
      if     ( num_discrete_points==0 )                       { num_discrete_points  = 2; }
      else if( tmp < 0.5*carmen_linemapping_params_global.merge_uniformly_distribute_dist ){ num_discrete_points += 1; }
      else                                                    { num_discrete_points += 2; }

      double weight_per_point = (double)s->weight / (double)num_discrete_points;
      for(double num_x=0.0; num_x<num_discrete_points-1; num_x += 1.0){
        carmen_point_t *p = new carmen_point_t;
        carmen_test_alloc(p);
        p->x = start_x + num_x*carmen_linemapping_params_global.merge_uniformly_distribute_dist;
        p->y = carmen_linemapping_get_y_coordinate_on_segment(s, p->x);
        p->theta = weight_per_point;
        points->add(p);
      }
      carmen_point_t *p = new carmen_point_t;
      carmen_test_alloc(p);
      p->x = end_x;
      p->y = carmen_linemapping_get_y_coordinate_on_segment(s, end_x);
      p->theta = weight_per_point;
      points->add(p);
    }
    else{
      double start_y, end_y;
      if( s->p1.y < s->p2.y ){ start_y = s->p1.y; end_y = s->p2.y; }
      else                   { start_y = s->p2.y; end_y = s->p1.y; }

      int num_discrete_points = (int)( dy / carmen_linemapping_params_global.merge_uniformly_distribute_dist );
      double tmp = dy - ((double)num_discrete_points)*carmen_linemapping_params_global.merge_uniformly_distribute_dist;
      if     ( num_discrete_points==0 )                       { num_discrete_points  = 2; }
      else if( tmp < 0.5 *carmen_linemapping_params_global.merge_uniformly_distribute_dist ){ num_discrete_points += 1; }
      else                                                    { num_discrete_points += 2; }

      double weight_per_point = (double)s->weight / (double)(num_discrete_points);
      for(double num_y=0.0; num_y<num_discrete_points-1; num_y += 1.0){
        carmen_point_t *p = new carmen_point_t;
        carmen_test_alloc(p);
        p->y = start_y + num_y*carmen_linemapping_params_global.merge_uniformly_distribute_dist;
        p->x = carmen_linemapping_get_x_coordinate_on_segment(s, p->y);
        p->theta = weight_per_point;
        points->add(p);
      }
      carmen_point_t *p = new carmen_point_t;
      carmen_test_alloc(p);
      p->y = end_y;
      p->x = carmen_linemapping_get_x_coordinate_on_segment(s, end_y);
      p->theta = weight_per_point;
      points->add(p);
    }
  }

  return points;

}


// euclidean distance of two points 'p1' and 'p2'
double carmen_linemapping_distance_point_point(const carmen_point_t *p1, 
					       const carmen_point_t *p2)
{
  return sqrt( carmen_square( p1->x - p2->x) + carmen_square( p1->y - p2->y) );
}



// returns 'true', if the point 'p' overlaps with the line segment 's'
int carmen_linemapping_overlap_point_linesegment(const carmen_linemapping_segment_t *s, 
						 const carmen_point_t *p)
{
  if( fabs(s->p2.x - s->p1.x) < carmen_linemapping_epsilon ){ // vertical line segment
    if( (s->p1.y <= p->y && s->p2.y >= p->y) || (s->p1.y >= p->y && s->p2.y <= p->y) ){ return true; }
    else{ return false; }
  }
  else if( fabs(s->p2.y - s->p1.y) < carmen_linemapping_epsilon ){ // horizontal line segment
    if( (s->p1.x <= p->x && s->p2.x >= p->x) || (s->p1.x >= p->x && s->p2.x <= p->x) ){ return true; }
    else{ return false; }
  }
  else{
    double m1 = (s->p2.y - s->p1.y) / (s->p2.x - s->p1.x);
    double b1 = s->p1.y - m1*s->p1.x;
    double m2 = -1.0/m1;
    double b2 = p->y - m2*p->x;
    double newX = (b2-b1) / (m1-m2);
    if( (s->p1.x <= newX && s->p2.x >= newX) || (s->p1.x >= newX && s->p2.x <= newX) ){ return true; }
    else{ return false; }
  }

}

// calculates the intersection 'cut_p' of two lines 's1' and 's2'
// returns 'true', if intersection exists
int carmen_linemapping_intersection_line_line(const carmen_linemapping_segment_t *s1, 
					      const carmen_linemapping_segment_t *s2, 
					      carmen_point_t *cut_p)
{
  double m1 = 0.0, m2 = 0.0, b1 = 0.0, b2 = 0.0;
  int vert_s1=false, horizont_s1=false, vert_s2=false, horizont_s2=false;

  if     ( fabs(s1->p2.x - s1->p1.x) < carmen_linemapping_epsilon ){ vert_s1    =true; } // s1 is vertical
  else if( fabs(s1->p2.y - s1->p1.y) < carmen_linemapping_epsilon ){ horizont_s1=true; } // s1 is horizontal
  else   { m1 = (s1->p2.y - s1->p1.y)/(s1->p2.x - s1->p1.x); b1 = s1->p1.y - m1*s1->p1.x; } // s1 otherwise

  
  if     ( fabs(s2->p2.x - s2->p1.x) < carmen_linemapping_epsilon ){ vert_s2    =true; } // s2 is vertical
  else if( fabs(s2->p2.y - s2->p1.y) < carmen_linemapping_epsilon ){ horizont_s2=true; } // s2 is horizontal
  else   { m2 = (s2->p2.y - s2->p1.y)/(s2->p2.x - s2->p1.x); b2 = s2->p1.y - m2*s2->p1.x; } // s2 otherwise

  if(vert_s1){
    if     (vert_s2)     { return false; }
    else if(horizont_s2) { cut_p->x = s1->p1.x; cut_p->y = s2->p1.y; }
    else                 { cut_p->x = s1->p1.x; cut_p->y = m2 * cut_p->x + b2; }
  }
  else if(horizont_s1){
    if(horizont_s2)  { return false; }
    else if(vert_s2) { cut_p->y = s1->p1.y; cut_p->x = s2->p1.x; }
    else             { cut_p->y = s1->p1.y; cut_p->x = (cut_p->y - b2) / m2; }
  }
  else{
    if(vert_s2)          { cut_p->x = s2->p1.x; cut_p->y = m1* cut_p->x + b1; }
    else if(horizont_s2) { cut_p->y = s2->p1.y; cut_p->x = (cut_p->y - b1)/m1; }
    else if( fabs(m1-m2) < carmen_linemapping_epsilon ) { return false;}
    else                 { cut_p->x = (b2-b1)/(m1-m2); cut_p->y = m1 * cut_p->x + b1; }
  }

  return true;

}

// returns a point at the line segment 's_over', which also lays on the orthogonal line to 's', going through one end point of 's'
carmen_point_t carmen_linemapping_get_point_on_segment(const carmen_linemapping_segment_t *s, 
						       const carmen_linemapping_segment_t *s_over)
{
  carmen_point_t p;

  carmen_linemapping_segment_t s_ortho1;
  s_ortho1.p1 = s->p1;

  carmen_linemapping_segment_t s_ortho2;
  s_ortho2.p1 = s->p2;


  if( fabs(s->p1.x - s->p2.x) < carmen_linemapping_epsilon ){  
    // vertical line segment
    s_ortho1.p2.x = s->p1.x + (s->p2.y - s->p1.y);
    s_ortho1.p2.y = s->p1.y;

    s_ortho2.p2.x = s->p2.x + (s->p1.y - s->p2.y);
    s_ortho2.p2.y = s->p2.y;
  }
  else if( fabs(s->p1.y - s->p2.y) < carmen_linemapping_epsilon ){  
    // horizontal line segment
    s_ortho1.p2.x = s->p1.x;
    s_ortho1.p2.y = s->p1.y + (s->p2.x - s->p1.x);

    s_ortho2.p2.x = s->p2.x;
    s_ortho2.p2.y = s->p2.y + (s->p1.x - s->p2.x);
  }
  else { 
    // other line segment
    double m = (s->p2.y - s->p1.y) / (s->p2.x - s->p1.x);
    double m_ortho = -1.0/m;
    double b_ortho1 = s->p1.y - m_ortho*s->p1.x;
    double b_ortho2 = s->p2.y - m_ortho*s->p2.x;

    s_ortho1.p2.x = s->p1.x + (s->p2.y - s->p1.y);
    s_ortho1.p2.y = m_ortho*s_ortho1.p2.x + b_ortho1;

    s_ortho2.p2.x = s->p2.x + (s->p1.y - s->p2.y);
    s_ortho2.p2.y = m_ortho*s_ortho2.p2.x + b_ortho2;
  }

  carmen_linemapping_intersection_line_line(s_over, &s_ortho1, &p);
  double dis_p1 = carmen_linemapping_distance_point_point(&s_over->p1, &p);
  double dis_p2 = carmen_linemapping_distance_point_point(&s_over->p2, &p);
  double length = carmen_linemapping_distance_point_point(&s_over->p1, &s_over->p2);

  if( dis_p1 > length || dis_p2 > length ){ 
    // 'p' does not lay on line segment 's_over'
    carmen_linemapping_intersection_line_line(s_over, &s_ortho2, &p);
  }

  return p;

}


// returns the acute angle between the two lines
double carmen_linemapping_angle_difference(const carmen_linemapping_segment_t *s1, 
					   const carmen_linemapping_segment_t *s2)
{
  double angle_s1, angle_s2; // angle is element of [0,PI]
  if ( s1->p1.y < s1->p2.y ) { 
    angle_s1 = atan2(s1->p2.y - s1->p1.y , s1->p2.x - s1->p1.x); 
  }
  else                     { 
    angle_s1 = atan2(s1->p1.y - s1->p2.y , s1->p1.x - s1->p2.x); 
  }
  
  if( s2->p1.y < s2->p2.y ){ 
    angle_s2 = atan2(s2->p2.y - s2->p1.y , s2->p2.x - s2->p1.x); 
  }
  else { 
    angle_s2 = atan2(s2->p1.y - s2->p2.y , s2->p1.x - s2->p2.x);
  }

  double div = fabs(angle_s1 - angle_s2);
  if( div > M_PI/2.0 )
    div = M_PI - div; 
  return div;
}



double          
carmen_linemapping_segment_length(const carmen_linemapping_segment_t *s) {
  return carmen_linemapping_distance_point_point(&(s->p1), &(s->p2));
}


double carmen_linemapping_distance_point_linesegment(const carmen_linemapping_segment_t *l, 
						     const carmen_point_t *p)
{
  // vertical linesegment
  if( fabs(l->p2.x - l->p1.x) < carmen_linemapping_epsilon ){
    if( (l->p1.y <= p->y && l->p2.y >= p->y) || 
	(l->p1.y >= p->y && l->p2.y <= p->y) ){
      return fabs(l->p1.x - p->x);
    }
    else{
      double dist1 = carmen_linemapping_distance_point_point(&l->p1, p);
      double dist2 = carmen_linemapping_distance_point_point(&l->p2, p);
      if( dist1 < dist2 ){ return dist1;}
      else               { return dist2;}
    }
  }
  // horizontal linesegment
  else if( fabs(l->p2.y - l->p1.y) < carmen_linemapping_epsilon ){
    if( (l->p1.x < p->x && l->p2.x > p->x) || 
	(l->p1.x > p->x && l->p2.x < p->x) ){
      return fabs(l->p1.y - p->y);
    }
    else{
      double dist1 = carmen_linemapping_distance_point_point(&l->p1, p);
      double dist2 = carmen_linemapping_distance_point_point(&l->p2, p);
      if(dist1<dist2)
	return dist1;
      else           
	return dist2;
    }
  }
  // other linesegments
  else{
    double m1 = (l->p2.y - l->p1.y) / (l->p2.x - l->p1.x);
    double b1 = l->p1.y - m1*l->p1.x;
    double m2 = -1.0/m1;
    double b2 = p->y - m2*p->x;
    carmen_point_t new_p;
    new_p.x = (b2-b1) / (m1-m2);
    new_p.y = m2*new_p.x + b2;
    
    if( (l->p1.x < new_p.x && l->p2.x > new_p.x) || 
	(l->p1.x > new_p.x && l->p2.x < new_p.x) ){
      return carmen_linemapping_distance_point_point(p, &new_p);
    }
    else{
      double dist1 = carmen_linemapping_distance_point_point(&l->p1, p);
      double dist2 = carmen_linemapping_distance_point_point(&l->p2, p);
      if(dist1<dist2)
	return dist1;
      else          
	return dist2;
    }
  }
}

double 
carmen_linemapping_distance_linesegment_linesegment(const carmen_linemapping_segment_t *l1,
						    const carmen_linemapping_segment_t *l2) {
  double min_dist = MAXDOUBLE;
  double dist = MAXDOUBLE;

  dist = carmen_linemapping_distance_point_linesegment(l1, &(l2->p1));
  if (min_dist > dist)
    min_dist = dist;

  dist = carmen_linemapping_distance_point_linesegment(l1, &(l2->p2));
  if (min_dist > dist)
    min_dist = dist;

  dist = carmen_linemapping_distance_point_linesegment(l2, &(l1->p1));
  if (min_dist > dist)
    min_dist = dist;

  dist = carmen_linemapping_distance_point_linesegment(l2, &(l1->p2));
  if (min_dist > dist)
    min_dist = dist;

  return min_dist;
}



