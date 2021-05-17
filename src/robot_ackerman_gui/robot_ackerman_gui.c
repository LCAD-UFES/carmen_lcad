/*********************************************************
 *
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

#include <carmen/carmen_graphics.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/robot_ackerman_interface.h>

#ifndef NO_JOYSTICK
#include <carmen/joyctrl.h>
#endif

#define       MAX_SAVED_POLYGONS        100

static carmen_robot_ackerman_laser_message front_laser, rear_laser;
static carmen_robot_ackerman_vector_status_message vector;
static carmen_base_ackerman_odometry_message odometry;

static int received_front_laser = 0;
static int received_rear_laser = 0;
static int new_front_laser = 0;
static int new_rear_laser = 0;

static int show_history = 0;
static int use_rear_laser = 1;
static int show_velocity = 1;
static int show_vector = 1;
static int gui_control = 1;
static double max_range = 25.0;

static GtkWidget *drawing_area;
static GdkGC *drawing_gc = NULL;

static GtkVScale *scaleBar;

static double connect_distance = 40;
static GdkColor gradient[MAX_SAVED_POLYGONS];

static int rectangular;
static double length, width;
static double max_v, max_phi;

static void redraw(void);

static int moving = 0;
static int rotating = 0;
static int mouse_down = 0;
static int draw_red = 0;
static double delta_angle;
static double displacement;

static double width_2, height_2;
static double scale;
static double max_plot_range;


#ifndef NO_JOYSTICK
static carmen_joystick_type joystick;
#endif
static void 
robot_frontlaser_handler(void)
{
  received_front_laser = 1;
  new_front_laser = 1;
  redraw();
}

static void
robot_rearlaser_handler(void)
{
  front_laser.laser_pose.x *= 100.0;
  front_laser.laser_pose.y *= 100.0;

  received_rear_laser = 1;
  new_rear_laser = 1;

  if (!received_front_laser)
    redraw();
}


static void
odometry_handler(void)
{
	carmen_robot_and_trailer_motion_command_t motion_command;
  
  if (!gui_control || !mouse_down)
    return;
  else
    return;
  
  if (!rotating && !moving)
  {
    motion_command.v = 0.0;
    motion_command.phi = 0.0;
    motion_command.time = 1.0;
    carmen_robot_ackerman_publish_motion_command(&motion_command, 1, carmen_get_time());
  }
  else 
  {
    if (!rotating)
      delta_angle = 0.0;
    if (!moving)
      displacement = 0.0;
    carmen_robot_ackerman_move_along_vector(displacement, delta_angle);
  }
}

static int 
initialize_robotgraph_ipc(void)
{
  carmen_robot_ackerman_subscribe_frontlaser_message
    (&front_laser, (carmen_handler_t)robot_frontlaser_handler,
     CARMEN_SUBSCRIBE_LATEST);

  if (use_rear_laser)
    carmen_robot_ackerman_subscribe_rearlaser_message
      (&rear_laser, (carmen_handler_t)robot_rearlaser_handler,
       CARMEN_SUBSCRIBE_LATEST);

  carmen_robot_ackerman_subscribe_vector_status_message
    (&vector, NULL, CARMEN_SUBSCRIBE_LATEST);
  
  memset(&vector, 0, sizeof(carmen_robot_ackerman_vector_status_message));

  carmen_base_ackerman_subscribe_odometry_message
    (&odometry, (carmen_handler_t)odometry_handler, CARMEN_SUBSCRIBE_LATEST);  

  return 0;
}

static void 
shutdown_robotgraph(int x)
{
  if(x == SIGINT) 
    {
#ifndef NO_JOYSTICK
      if (joystick.initialized)
	carmen_close_joystick(&joystick);
#endif
      carmen_ipc_disconnect();
      printf("Disconnected from robot.\n");
      exit(1);
    }
}

static gint 
updateIPC(gpointer *data __attribute__ ((unused))) 
{
  carmen_ipc_sleep(0.01);
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  return 1;
}

static void 
save_postscript_screenshot(void)
{
  static int screenshot_number = 0;
  char filename[100];
  carmen_ps_doc_p doc;
  float *laser_x, *laser_y;
  float rect_x[4], rect_y[4];
  int i;
  double range, theta, coswidth, coslength, sinlength, sinwidth;

  sprintf(filename, "rg_screenshot%03d.ps", screenshot_number);
  doc = carmen_ps_open(filename, 4, 3, CARMEN_GENERATE_PS);
  if(doc == NULL) {
    carmen_warn("Error: Could not generate .ps document.\n");
    return;
  }
  carmen_ps_set_jointype(doc, CARMEN_PS_BEVELJOIN);
  carmen_ps_set_color(doc, 30, 144, 255);
  carmen_ps_draw_rectangle(doc, 1, 0, 0, doc->width, doc->height);

  laser_x = (float *)calloc(front_laser.num_readings, sizeof(float));
  carmen_test_alloc(laser_x);
  laser_y = (float *)calloc(front_laser.num_readings, sizeof(float));
  carmen_test_alloc(laser_y);

  for(i = 0; i < front_laser.num_readings; i++) {
    theta = front_laser.laser_pose.theta + front_laser.config.start_angle + 
      ((double)i) *  front_laser.config.angular_resolution ;
    range = front_laser.range[i];
    laser_x[i] = front_laser.laser_pose.x - front_laser.robot_pose.x + 
      cos(theta) * range;
    laser_y[i] = front_laser.laser_pose.y - front_laser.robot_pose.y +
      sin(theta) * range;
    laser_x[i] = doc->width / 2.0 + laser_x[i] / 500.0 * 2.0;
    laser_y[i] = doc->height / 2.0 + laser_y[i] / 500.0 * 2.0;
  }
  carmen_ps_set_color(doc, 255, 255, 255);
  carmen_ps_draw_poly(doc, 1, laser_x, laser_y, front_laser.num_readings, 0);
  carmen_ps_draw_poly(doc, 0, laser_x, laser_y, front_laser.num_readings, 0);

  coslength = cos(front_laser.robot_pose.theta) / 2.0 * length;
  coswidth = cos(front_laser.robot_pose.theta) / 2.0 * width;
  sinlength = sin(front_laser.robot_pose.theta) / 2.0 * length;
  sinwidth = sin(front_laser.robot_pose.theta) / 2.0 * width;
  
  rect_x[0] = doc->width / 2.0 + (coslength - sinwidth) / 500.0 * 2.0; 
  rect_y[0] = doc->height / 2.0 + (sinlength + coswidth) / 500.0 * 2.0;
  rect_x[1] = doc->width / 2.0 + (coslength + sinwidth) / 500.0 * 2.0;
  rect_y[1] = doc->height / 2.0 + (sinlength - coswidth) / 500.0 * 2.0;
  rect_x[3] = doc->width / 2.0 - (coslength + sinwidth) / 500.0 * 2.0;
  rect_y[3] = doc->height / 2.0 - (sinlength - coswidth) / 500.0 * 2.0;
  rect_x[2] = doc->width / 2.0 - (coslength - sinwidth) / 500.0 * 2.0;
  rect_y[2] = doc->height / 2.0 - (sinlength + coswidth) / 500.0 * 2.0;
  
  carmen_ps_set_color(doc, 255, 255, 255);
  carmen_ps_draw_poly(doc, 1, rect_x, rect_y, 4, 1);
  carmen_ps_draw_poly(doc, 0, rect_x, rect_y, 4, 1);
  carmen_ps_draw_line(doc, doc->width / 2.0, doc->height / 2.0, 
	       doc->width / 2.0 + length * 0.75 / 500.0 * 2.0 * 
	       cos(front_laser.robot_pose.theta),
	       doc->height / 2.0 + length * 0.75 / 500.0 * 2.0 * 
	       sin(front_laser.robot_pose.theta));
  carmen_ps_close(doc);
  carmen_verbose("Generated %s\n", filename);
  screenshot_number++;
}

static void
set_rotating(int x, int y)
{
  double delta_x, delta_y;
  double distance;
  double destination_angle;

  delta_x = x - width_2;
  delta_y = height_2 - y;
  destination_angle = atan2(delta_y, delta_x);
  delta_angle = carmen_normalize_theta(destination_angle - odometry.theta);

  //carmen_warn("dest angle: %f delta_angle: %f (odom: %f)\n", 
  //carmen_radians_to_degrees(destination_angle), carmen_radians_to_degrees(delta_angle),
  //carmen_radians_to_degrees(odometry.theta));

  distance = hypot(delta_x, delta_y);
  
  if (distance < length*scale || fabs(carmen_radians_to_degrees(destination_angle)) < 1.0) 
    {
      destination_angle = 0;
      rotating = 0;
    } 
  else 
    {
      rotating = 1;
    }
}

static void
set_moving(int x, int y)
{
  double delta_x, delta_y;
  double distance;

  delta_x = x - width_2;
  delta_y = height_2 - y;
  
  distance = hypot(delta_x, delta_y);

  if (!moving && distance < length/2*scale) 
    moving = 1;
  else if (moving)
    {
      displacement = distance/scale;
      if (displacement > length)
	displacement -= length;
      else if (displacement < -length)
	displacement += length;
      else 
	displacement = 0.0;	
    }
}

static gint 
button_press_event(GtkWidget *widget __attribute__ ((unused)), 
		   GdkEventButton *event)
{
  if (event->state & GDK_CONTROL_MASK) 
    {
      if (received_front_laser) 
	save_postscript_screenshot();
      return 1;
    } 

  mouse_down = 1;
  
  width_2 = drawing_area->allocation.width / 2;
  height_2 = drawing_area->allocation.height / 2;
  max_plot_range = MAX(width_2, height_2);
  scale = max_plot_range / (double)max_range;

  set_rotating(event->x, event->y);
  set_moving(event->x, event->y);

  if (!rotating)
    delta_angle = 0.0;
  if (!moving)
    displacement = 0.0;
  //    carmen_warn("Moving along vector %f %f (%d %d)\n", displacement, 
  //     carmen_radians_to_degrees(delta_angle),
  //		 rotating, moving);
  carmen_robot_ackerman_move_along_vector(displacement, delta_angle);

  return 1;
}

static gint 
button_release_event(GtkWidget *widget __attribute__ ((unused)), 
		     GdkEventButton *event __attribute__ ((unused)))
{
	carmen_robot_and_trailer_motion_command_t motion_command;
  
  moving = 0;
  rotating = 0;
  mouse_down = 0;

  motion_command.v = 0.0;
  motion_command.phi = 0.0;
  motion_command.time = 1.0;
  carmen_robot_ackerman_publish_motion_command(&motion_command, 1, carmen_get_time());
  
  return 1;
}

static gint
key_press_event(GtkWidget *widget __attribute__ ((unused)), 
		GdkEventKey *key)
{
  double v, phi;
  int err;
  static double time_since_last_key_command = -1;
  static double current_commanded_v = 0, current_commanded_phi = 0; 
  carmen_robot_and_trailer_motion_command_t motion_command;

  if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK))
    shutdown_robotgraph(SIGINT);
  
  if (key->keyval == 65365) { // Page Up
    max_range = max_range+2;
    //carmen_param_set_double("maxrange", max_range, NULL);
  }

  if (key->keyval == 65366) { // Page Up
    max_range = max_range-2;
    if (max_range < .5)
      max_range = .5;
    //carmen_param_set_double("maxrange", max_range, NULL);
  }

  if (GDK_CONTROL_MASK & key->state || 
      GDK_SHIFT_MASK & key->state || 
      GDK_MOD1_MASK & key->state || 
      key->keyval > 255)
    return 1;

  if (toupper(key->keyval) == 'H')
    show_history = !show_history;

  if (toupper(key->keyval) == 'V')
    show_velocity = !show_velocity;

  if (toupper(key->keyval) == 'T')
    show_vector = !show_vector;

  if (toupper(key->keyval) == 'R')
    use_rear_laser = !use_rear_laser;

  if (toupper(key->keyval) == 'C')
    gui_control = !gui_control;

  //  if (toupper(key->keyval) == 'Q')
  //    return 1;

  if (!gui_control)
    return 1;

  err = carmen_keyboard_control(key->keyval, max_v, max_phi, &v, &phi);
  if(err < 0)
    shutdown_robotgraph(SIGINT);
  else {
    // Only send the command if it's been longer than 1/10 of a second since
    // we sent a command, or  if the new command is substantially different
    // from our last command. 
    if (carmen_get_time() - time_since_last_key_command > .1 ||
	fabs(v - current_commanded_v) > .1 ||
	fabs(phi - current_commanded_phi) > carmen_radians_to_degrees(5)) {
      motion_command.v = v;
      motion_command.phi = phi;
      motion_command.time = 1.0;
      carmen_robot_ackerman_publish_motion_command(&motion_command, 1, carmen_get_time());
      current_commanded_v = v;
      current_commanded_phi = phi; 
      time_since_last_key_command = carmen_get_time();
    } 
 }

  return 1;
}

static gint 
key_release_event(GtkWidget *widget __attribute__ ((unused)), 
		  GdkEventButton *key __attribute__ ((unused)))
{
  return 1;
}

static gint 
motion_event(GtkWidget *widget __attribute__ ((unused)), 
	     GdkEventMotion *event)
{
  int x, y;
  GdkModifierType state;
  double delta_x, delta_y;
  double distance;

  if (event->is_hint) 
    {
      gdk_window_get_pointer (event->window, &x, &y, &state);
    }
  else
    {
      x = event->x;
      y = event->y;
      state = event->state;
    }

  if (mouse_down) 
    {
      set_rotating(x, y);
      if (moving)
	set_moving(x, y);

      if (!rotating)
	delta_angle = 0.0;
      if (!moving)
	displacement = 0.0;
      //    carmen_warn("Moving along vector %f %f (%d %d)\n", displacement, 
      //     carmen_radians_to_degrees(delta_angle),
      //		 rotating, moving);
      carmen_robot_ackerman_move_along_vector(displacement, delta_angle);
    }
  else 
    {
      delta_x = x - width_2;
      delta_y = height_2 - y;
		
      distance = hypot(delta_x, delta_y);
		
      if (distance < length/2*scale) 
	draw_red = 1;
      else
	draw_red = 0;
    }
	
  return 1;
}

static gint 
Expose_Event(GtkWidget *widget __attribute__ ((unused)), 
	     GdkEventExpose *event __attribute__ ((unused))) 
{
  redraw();
  return 1;
}

static void
setup_colors(void) 
{  
  int i, r, g, b;

  carmen_graphics_setup_colors();
  
  for(i = 0; i < MAX_SAVED_POLYGONS; i++) {
    r = 30 + (220 - 30) * i / (double)MAX_SAVED_POLYGONS;
    g = 144 + (220 - 144) * i / (double)MAX_SAVED_POLYGONS;
    b = 255 + (220 - 255) * i / (double)MAX_SAVED_POLYGONS;
    
    r = 30 + (255 - 30) * i / (double)MAX_SAVED_POLYGONS;
    g = 144 + (255 - 144) * i / (double)MAX_SAVED_POLYGONS;
    b = 255 + (255 - 255) * i / (double)MAX_SAVED_POLYGONS;
    gradient[i] = carmen_graphics_add_color_rgb(r, g, b);
  }
  
  if(drawing_gc == NULL) 
    drawing_gc = gdk_gc_new(drawing_area->window);
  
  gdk_gc_set_line_attributes(drawing_gc, 2, GDK_LINE_SOLID,
			     GDK_CAP_NOT_LAST, GDK_JOIN_MITER);
}

static void 
range_to_xy(carmen_robot_ackerman_laser_message *laser, GdkPoint *laser_poly,
	    double *laser_dist)
{
  int i;
  double theta;

  for(i = 0; i < laser->num_readings; i++) {


/*     theta = laser->laser_pose.theta +  */
/*       M_PI * i / (double)(laser->num_readings - 1) - M_PI / 2.0; */

    theta = laser->laser_pose.theta + laser->config.start_angle +
          (double)i * laser->config.angular_resolution ;


    laser_poly[i].x = width_2 + ((laser->laser_pose.x - 
				  laser->robot_pose.x) + 
				 cos(theta) * laser->range[i]) * scale;
    laser_poly[i].y = height_2 - ((laser->laser_pose.y - 
				   laser->robot_pose.y) + 
				  sin(theta) * laser->range[i]) * scale;
    if(i > 0)
      laser_dist[i] = hypot(laser_poly[i - 1].x - laser_poly[i].x, 
			    laser_poly[i - 1].y - laser_poly[i].y);
  }
  laser_poly[laser->num_readings].x = width_2 + 
    (laser->laser_pose.x - laser->robot_pose.x) * scale;
  laser_poly[laser->num_readings].y = height_2 - 
    (laser->laser_pose.y - laser->robot_pose.y) * scale;
}

static void 
offset_polygon(GdkPoint *poly1, GdkPoint *poly2, int num_points,
	       double x_offset, double y_offset)
{
  int i;
  
  for(i = 0; i < num_points; i++) {
    poly2[i].x = poly1[i].x + x_offset;
    poly2[i].y = poly1[i].y + y_offset;
  }
}

static void 
check_capacity(int new_laser, int new_size, double **laser_dist, 
	       int *previous_size, GdkPoint **laser_poly, 
	       GdkPoint *free_polygons[])
{
  int i;

  if (!new_laser)
    return;

  if (*previous_size == new_size)
    return;

  (*laser_dist) = (double *)realloc
    ((*laser_dist), sizeof(double) * (new_size + 1));
  carmen_test_alloc(*laser_dist);

  (*laser_poly) = (GdkPoint *)realloc
    ((*laser_poly), sizeof(GdkPoint) * (new_size + 1));

  carmen_test_alloc(*laser_poly);

  for(i = 0; i < MAX_SAVED_POLYGONS; i++)
    {
      free_polygons[i] = (GdkPoint *)realloc
	(free_polygons[i], sizeof(GdkPoint) * (new_size + 1));
      carmen_test_alloc(free_polygons[i]);
    }

  *previous_size = new_size;
}

static void 
update_polygons(int received_laser, carmen_robot_ackerman_laser_message *laser_msg,
		GdkPoint *laser_poly, double *laser_dist, 
		carmen_point_t *last_odom, carmen_point_t *current_odom)
{
  if(!received_laser)
    return;

  /* compute x, y positions of laser points */
  range_to_xy(laser_msg, laser_poly, laser_dist);

  /* update odometry */
  *last_odom = *current_odom;
  current_odom->x = laser_msg->laser_pose.x;
  current_odom->y = laser_msg->laser_pose.y;
  current_odom->theta = laser_msg->laser_pose.theta;

}

static void 
update_history(int received_laser, carmen_point_t *last_odom, 
	       carmen_point_t *current_odom, 
	       int *current_polygon, GdkPoint *laser_poly, int poly_size,
	       GdkPoint *polygon_history[], carmen_point_t *odometry_history,
	       int *polygon_active)
{
  if (!show_history || !received_laser)
    return;

  /* If necessary, add polygon */

  if (hypot(current_odom->x - last_odom->x, current_odom->y - last_odom->y) > 0.0 ||
      fabs(last_odom->theta - current_odom->theta) > 0.0) 
    {
      memcpy(polygon_history[*current_polygon], 
	     laser_poly, sizeof(GdkPoint) * poly_size);
      odometry_history[*current_polygon] = *current_odom;
      polygon_active[*current_polygon] = 1;
      (*current_polygon)++;
      if(*current_polygon == MAX_SAVED_POLYGONS)
	*current_polygon = 0;
    }
}

static void draw_laser_data(GdkPoint *laser_poly, int polygon_size, 
			    GdkPixmap *pixmap, int current_polygon, 
			    GdkPoint *free_polygons[], 
			    carmen_point_t *odometry_history, 
			    int *polygon_active, carmen_point_t *current_odom)
{
  int i;
  int which;

  if(!show_history && received_front_laser) {
    gdk_gc_set_foreground(drawing_gc, &carmen_white);
    gdk_draw_polygon(pixmap, drawing_gc, TRUE, laser_poly, polygon_size);

    return;
  }

  /* draw all freespace polygons */
  for(i = 0; i < MAX_SAVED_POLYGONS; i++) {
    which = (current_polygon+i) % MAX_SAVED_POLYGONS;
    if(!polygon_active[which])
      continue;

    gdk_gc_set_foreground(drawing_gc, &(gradient[i]));
    offset_polygon(free_polygons[which], laser_poly, polygon_size,
		   (odometry_history[which].x - current_odom->x) * scale,
		   -(odometry_history[which].y - current_odom->y) * scale);
    gdk_draw_polygon(pixmap, drawing_gc, TRUE, laser_poly, polygon_size);
  }
}


static void draw_laser_endpoints(GdkPoint *laser_poly, 
				 carmen_robot_ackerman_laser_message *laser_msg,
				 double *laser_dist, GdkPixmap *pixmap)
{
  int i;

  /* draw blue laser points and lines - safe according to robot */
  gdk_gc_set_foreground(drawing_gc, &carmen_blue);
  for(i = 0; i < laser_msg->num_readings; i++) {
    if(laser_msg->tooclose[i] == 1) continue;
    if(i > 0 && laser_dist[i] < connect_distance)
      gdk_draw_line(pixmap, drawing_gc, 
		    laser_poly[i - 1].x, laser_poly[i - 1].y, 
		    laser_poly[i].x, laser_poly[i].y);
    else
      gdk_draw_arc(pixmap, drawing_gc, TRUE, 
		   laser_poly[i].x - 2, 
		   laser_poly[i].y - 2, 4, 4, 0, 360*64);
  }
  
  /* draw red laser points and lines - possible collisions */
  gdk_gc_set_foreground (drawing_gc, &carmen_red);
  for(i = 0; i < laser_msg->num_readings; i++) {
    if(laser_msg->tooclose[i] < 1) continue;
    if(i > 0 && laser_dist[i] < connect_distance)
      gdk_draw_line(pixmap, drawing_gc, 
		    laser_poly[i - 1].x, laser_poly[i - 1].y, 
		    laser_poly[i].x, laser_poly[i].y);
    else
      gdk_draw_arc(pixmap, drawing_gc, TRUE, 
		   laser_poly[i].x - 2, 
		   laser_poly[i].y - 2, 4, 4, 0, 360*64);
  }

}


static void 
draw_robot(GdkPixmap *pixmap, carmen_robot_ackerman_laser_message *laser_msg)
{
  GdkPoint rect_robot[4];
  GdkPoint p1, p2;
  GdkPoint top_left;
  int x_1, y_1;
  double coswidth, coslength, sinwidth, sinlength;
  double radius, angle;
  int start;

  /* Draw turning axis as red plus */
  gdk_gc_set_foreground (drawing_gc, &carmen_red);
  x_1 = width_2 + sin(laser_msg->robot_pose.theta) * 
    laser_msg->turn_axis * scale;
  y_1 = height_2 + cos(laser_msg->robot_pose.theta) * 
    laser_msg->turn_axis * scale;
  gdk_draw_line(pixmap, drawing_gc, x_1 - 5, y_1, x_1 + 5, y_1);
  gdk_draw_line(pixmap, drawing_gc, x_1, y_1 - 5, x_1, y_1 + 5);
  
  /* Draw robot */
  gdk_gc_set_foreground(drawing_gc, &carmen_light_grey);
  if(!rectangular) {
    gdk_draw_arc(pixmap, drawing_gc, TRUE, width_2 - length / 2.0 * scale,
		 height_2 - length / 2.0 * scale, 
		 length * scale, length * scale, 0, 360 * 64);
    if (draw_red)
      gdk_gc_set_foreground(drawing_gc, &carmen_red);
    else
      gdk_gc_set_foreground(drawing_gc, &carmen_black);
    gdk_draw_arc(pixmap, drawing_gc, FALSE, width_2 - length / 2.0 * scale, 
		 height_2 - length / 2.0 * scale, 
		 length * scale, length * scale, 0, 360 * 64);  
  }
  else {
    coslength = cos(laser_msg->robot_pose.theta) / 2.0 * scale * length;
    coswidth = cos(laser_msg->robot_pose.theta) / 2.0 * scale * width;
    sinlength = sin(laser_msg->robot_pose.theta) / 2.0 * scale * length;
    sinwidth = sin(laser_msg->robot_pose.theta) / 2.0 * scale * width;
    
    rect_robot[0].x = width_2 + coslength - sinwidth; 
    rect_robot[0].y = height_2 - sinlength - coswidth;
    rect_robot[1].x = width_2 + coslength + sinwidth;
    rect_robot[1].y = height_2 - sinlength + coswidth;
    rect_robot[2].x = width_2 - coslength + sinwidth;
    rect_robot[2].y = height_2 + sinlength + coswidth;
    rect_robot[3].x = width_2 - coslength - sinwidth;
    rect_robot[3].y = height_2 + sinlength - coswidth;
    
   gdk_draw_polygon(pixmap, drawing_gc, TRUE, rect_robot, 4);
    if (draw_red)
      gdk_gc_set_foreground(drawing_gc, &carmen_red);
    else
      gdk_gc_set_foreground(drawing_gc, &carmen_black);
    gdk_draw_polygon(pixmap, drawing_gc, FALSE, rect_robot, 4);
  }
  gdk_draw_line(pixmap, drawing_gc, width_2, height_2,
		width_2 + cos(laser_msg->robot_pose.theta + laser_msg->phi) *
		length / 2.0 * scale,
		height_2 - sin(laser_msg->robot_pose.theta + laser_msg->phi) *
		length / 2.0 * scale);
  if (show_velocity && 
      fabs(laser_msg->v) < 0.001 && fabs(laser_msg->phi) > .00001)
    {
      gdk_gc_set_foreground(drawing_gc, &carmen_green);
      gdk_draw_arc(pixmap, drawing_gc, FALSE, width_2 - length/2.0*scale*1.1, 
		   height_2 - length / 2.0 * scale*1.1, 
		   length * scale*1.1, length * scale*1.1, 0, 360 * 64);      
    }
  else if (show_velocity && 
	   fabs(laser_msg->v) > 0.001 && fabs(laser_msg->phi) > 0.001)
    {
      gdk_gc_set_foreground(drawing_gc, &carmen_green);

      radius = 1.0/carmen_normalize_theta(laser_msg->phi);

      if (radius > 0) {
	top_left.x = width_2 +cos(laser_msg->robot_pose.theta+M_PI/2)*  (radius+width/2.0)*scale-radius*scale;
	top_left.y = height_2-sin(laser_msg->robot_pose.theta+M_PI/2)* (radius+width/2.0)*scale-radius*scale;
	angle = carmen_normalize_theta(laser_msg->robot_pose.theta-M_PI/2);
      } else {
	radius = fabs(radius);
	top_left.x = width_2+cos(laser_msg->robot_pose.theta-M_PI/2)*
	  (radius+width/2.0)*scale-radius*scale;
	top_left.y = height_2-sin(laser_msg->robot_pose.theta-M_PI/2)*
	  (radius+width/2.0)*scale-radius*scale;
	angle = carmen_normalize_theta(laser_msg->robot_pose.theta);
      }

      if (angle < 0)
	angle += 2*M_PI;
      start = carmen_trunc(carmen_radians_to_degrees(angle))*64;
      gdk_draw_arc(pixmap, drawing_gc, FALSE, top_left.x, top_left.y,
		   radius*2*scale, radius*2*scale, start, 90*64);

      radius = 1.0/carmen_normalize_theta(laser_msg->phi);

      if (radius > 0) {
	radius += width;
	top_left.x = width_2+cos(laser_msg->robot_pose.theta+M_PI/2)*
	  (radius-width/2.0)*scale-radius*scale;
	top_left.y = height_2-sin(laser_msg->robot_pose.theta+M_PI/2)*
	  (radius-width/2.0)*scale-radius*scale;
	angle = carmen_normalize_theta(laser_msg->robot_pose.theta-M_PI/2);
      } else {
	radius = fabs(radius) + width;
	top_left.x = width_2+cos(laser_msg->robot_pose.theta-M_PI/2)*
	  (radius-width/2.0)*scale-radius*scale;
	top_left.y = height_2-sin(laser_msg->robot_pose.theta-M_PI/2)*
	  (radius-width/2.0)*scale-radius*scale;
	angle = carmen_normalize_theta(laser_msg->robot_pose.theta);
      }

      if (angle < 0)
	angle += 2*M_PI;
      start = carmen_trunc(carmen_radians_to_degrees(angle))*64;
      gdk_draw_arc(pixmap, drawing_gc, FALSE, top_left.x, top_left.y,
		   radius*2*scale, radius*2*scale, start, 90*64);


    } else if (show_velocity && fabs(laser_msg->v) > 0.001) {
      gdk_gc_set_foreground(drawing_gc, &carmen_green);
      p1.x = width_2 + cos(laser_msg->robot_pose.theta+M_PI/2)*width/2.0*scale*1.2;
      p1.y = height_2 - sin(laser_msg->robot_pose.theta+M_PI/2)*width/2.0*scale*1.2;

      p2 = p1;
      p2.x += cos(laser_msg->robot_pose.theta)*100*scale;
      p2.y -= sin(laser_msg->robot_pose.theta)*100*scale;
      gdk_draw_line(pixmap, drawing_gc, p1.x, p1.y, p2.x, p2.y);

      p1.x = width_2 + cos(laser_msg->robot_pose.theta-M_PI/2)*width/2.0*scale*1.2;
      p1.y = height_2 - sin(laser_msg->robot_pose.theta-M_PI/2)*width/2.0*scale*1.2;
      p2 = p1;
      p2.x += cos(laser_msg->robot_pose.theta)*100*scale;
      p2.y -= sin(laser_msg->robot_pose.theta)*100*scale;
      gdk_draw_line(pixmap, drawing_gc, p1.x, p1.y, p2.x, p2.y);
    }
  if (show_vector && fabs(vector.vector_distance) > 0) {
    gdk_gc_set_foreground(drawing_gc, &carmen_blue);
    p1.x = width_2;
    p1.y = height_2;
    
    p2 = p1;
    p2.x += cos(laser_msg->robot_pose.theta+vector.vector_angle)*
      vector.vector_distance*scale;
    p2.y -= sin(laser_msg->robot_pose.theta+vector.vector_angle)*
      vector.vector_distance*scale;    
    gdk_draw_line(pixmap, drawing_gc, p1.x, p1.y, p2.x, p2.y);

    p2.x -= 5;
    p2.y -= 5;
    gdk_draw_arc(pixmap, drawing_gc, FALSE, p2.x, p2.y,
		 10, 10, 0, 360*64);    
  }
}

static void 
redraw(void)
{
  static GdkPixmap *pixmap = NULL;

  static int first = 1;
  static GdkPoint *front_free_polygons[MAX_SAVED_POLYGONS];
  static GdkPoint *rear_free_polygons[MAX_SAVED_POLYGONS];
  static GdkPoint *front_laser_poly = NULL, *rear_laser_poly = NULL;
  static double *front_laser_dist = NULL, *rear_laser_dist = NULL;
  static int num_front_points = -1;
  static int num_rear_points = -1;
  static carmen_point_t front_laser_odometry[MAX_SAVED_POLYGONS];
  static carmen_point_t rear_laser_odometry[MAX_SAVED_POLYGONS];
  static int front_polygon_active[MAX_SAVED_POLYGONS];
  static int rear_polygon_active[MAX_SAVED_POLYGONS];
  static int current_front_polygon = 0, current_rear_polygon = 0;  

  static carmen_point_t last_front_odom, current_front_odom;
  static carmen_point_t last_rear_odom, current_rear_odom;
  static GdkFont *font;

  static char filename[1024];
  static int screenshot_count = 0;

  /* clear all polygons */
  if(first) {
    memset(front_free_polygons, 0, MAX_SAVED_POLYGONS*sizeof(GdkPoint *));
    memset(rear_free_polygons, 0, MAX_SAVED_POLYGONS*sizeof(GdkPoint *));
    memset(front_polygon_active, 0, MAX_SAVED_POLYGONS*sizeof(int));
    memset(rear_polygon_active, 0, MAX_SAVED_POLYGONS*sizeof(int));
    font = gdk_font_load ("-adobe-times-bold-r-*-*-14-*-*-*-*-*-*-*");
    if (font == NULL)
      font = gdk_font_load ("-*-fixed-*-r-*-*-14-*-*-*-*-*-*-*");

    first = 0;
  }

  /* Make sure data structures are all the right size. */

  check_capacity(new_front_laser, front_laser.num_readings, &front_laser_dist, 
		 &num_front_points, &front_laser_poly, front_free_polygons);

  check_capacity(new_rear_laser, rear_laser.num_readings, &rear_laser_dist, 
		 &num_rear_points, &rear_laser_poly, rear_free_polygons);

  /* setup graphics parameters */
  setup_colors();
  width_2 = drawing_area->allocation.width / 2;
  height_2 = drawing_area->allocation.height / 2;
  if (width_2 > height_2) 
    max_plot_range = height_2;
  else
    max_plot_range = width_2;
  scale = max_plot_range / (double)max_range;
  if (pixmap == NULL)
    pixmap = gdk_pixmap_new(drawing_area->window, 
			    drawing_area->allocation.width,
			    drawing_area->allocation.height, -1);
  if (pixmap == NULL)
    return;

  /* erase window with light blue */
  gdk_gc_set_foreground(drawing_gc, &carmen_light_blue);
  gdk_draw_rectangle(pixmap, drawing_gc, TRUE, 0, 0, 
		     drawing_area->allocation.width, 
		     drawing_area->allocation.height);
  
  if (!received_front_laser && !received_rear_laser && font != NULL)
    {
      gdk_gc_set_foreground(drawing_gc, &carmen_white);
      gdk_draw_string(pixmap, font, drawing_gc, 
		      drawing_area->allocation.width/2-50, 
		      drawing_area->allocation.height/2, 
		      "Waiting for data");
    }

  /* fill in polygon data structures */

  if (received_front_laser)
    {
      update_polygons(new_front_laser, &front_laser, front_laser_poly, 
		      front_laser_dist, &last_front_odom, &current_front_odom);
      update_history(new_front_laser, &last_front_odom, &current_front_odom, 
		     &current_front_polygon, front_laser_poly, 
		     front_laser.num_readings+1, front_free_polygons, 
		     front_laser_odometry, front_polygon_active);

      draw_laser_data(front_laser_poly, front_laser.num_readings+1, pixmap,
		      current_front_polygon, front_free_polygons, 
		      front_laser_odometry, front_polygon_active,
		      &current_front_odom);
      
      draw_laser_endpoints(front_laser_poly, &front_laser, 
			   front_laser_dist, pixmap);
      
      new_front_laser = 0;
    }

  if (received_rear_laser)
    {
      update_polygons(new_rear_laser, &rear_laser, rear_laser_poly, 
		      rear_laser_dist, &last_rear_odom, &current_rear_odom);
      update_history(new_rear_laser, &last_rear_odom, &current_rear_odom, 
		     &current_rear_polygon, rear_laser_poly, 
		     rear_laser.num_readings+1, rear_free_polygons, 
		     rear_laser_odometry, rear_polygon_active);
      
      draw_laser_data(rear_laser_poly, rear_laser.num_readings+1, pixmap,
		      current_rear_polygon, rear_free_polygons, 
		      rear_laser_odometry, rear_polygon_active,
		      &current_rear_odom);

      draw_laser_endpoints(rear_laser_poly, &rear_laser, 
			   rear_laser_dist, pixmap);
      
      new_rear_laser = 0;
    }

  /* draw robot from front laser information */
  if(received_front_laser)
    draw_robot(pixmap, &front_laser);
  else if (received_rear_laser)
    draw_robot(pixmap, &rear_laser);

  /* In case of screenshots break glass */

  if(0 && received_front_laser) {
    sprintf(filename, "robotgui%02d.png", screenshot_count++);
    carmen_graphics_write_pixmap_as_png(pixmap, filename, 0, 0, 
					drawing_area->allocation.width,
					drawing_area->allocation.height);
  }

  /* udpate the whole window */

  gdk_draw_pixmap(drawing_area->window, 
		  drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
                  pixmap, 0, 0, 0, 0, 
		  drawing_area->allocation.width, 
		  drawing_area->allocation.height);

}

#ifndef NO_JOYSTICK
static int
Joystick_Event(GtkWidget *widget __attribute__ ((unused)),
	       gpointer data __attribute__ ((unused)))
{
  double v, phi;
  static double last_update = 0;
  carmen_robot_and_trailer_motion_command_t motion_command;

  if (!joystick.initialized)
    return FALSE;
  
  if(carmen_get_joystick_state(&joystick) >= 0 || 
     carmen_get_time() - last_update > 0.5) {
    carmen_joystick_control(&joystick, max_v, max_phi, &v, &phi);
    motion_command.v = v;
    motion_command.phi = phi;
    motion_command.time = 1.0;
    carmen_robot_ackerman_publish_motion_command(&motion_command, 1, carmen_get_time());
    last_update = carmen_get_time();

  }
  return TRUE;
}

static void scaleHandler(GtkRange *range) {
	max_range = gtk_range_get_value(range);
}

#endif
static void 
start_graphics(int argc, char *argv[]) 
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);

  main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (main_window), "Robot Graph");
  
  scaleBar = (GtkVScale *) gtk_vscale_new_with_range(1, 40, 0.5);
  gtk_widget_set_usize ((GtkWidget *)scaleBar, 30, 500);
  gtk_range_set_value((GtkRange *)scaleBar, max_range);
  gtk_signal_connect(GTK_OBJECT(scaleBar), "value-changed",
		     (GtkSignalFunc)scaleHandler, NULL);

  
   
  drawing_area = gtk_drawing_area_new ();
  gtk_widget_set_usize (drawing_area, 500, 500);
  
  GtkBox *box;
  box = (GtkBox*)gtk_hbox_new(FALSE, 2);
  
  
  gtk_box_pack_start(box, drawing_area, TRUE, FALSE, 0);
  gtk_box_pack_start(box, (GtkWidget *)scaleBar, TRUE, FALSE, 0);
  
  gtk_container_add(GTK_CONTAINER(main_window), (GtkWidget *)box);
  
  gtk_signal_connect(GTK_OBJECT(drawing_area), "expose_event",
		     (GtkSignalFunc)Expose_Event, NULL);
  gtk_signal_connect(GTK_OBJECT(drawing_area), "button_press_event",
		     (GtkSignalFunc)button_press_event, NULL);
  gtk_signal_connect(GTK_OBJECT(drawing_area), "button_release_event",
  		     (GtkSignalFunc)button_release_event, NULL);
  gtk_signal_connect(GTK_OBJECT(drawing_area), "motion_notify_event",
		     (GtkSignalFunc)motion_event, NULL);
  gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
		     (GtkSignalFunc)key_press_event, NULL);
  gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
		     (GtkSignalFunc)key_release_event, NULL);
  
  gtk_widget_add_events(drawing_area,  GDK_EXPOSURE_MASK 
			| GDK_BUTTON_PRESS_MASK 
			| GDK_BUTTON_RELEASE_MASK 
			| GDK_POINTER_MOTION_MASK
			| GDK_POINTER_MOTION_HINT_MASK
			| GDK_KEY_PRESS_MASK
			| GDK_KEY_RELEASE_MASK);
  
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  gtk_widget_show( (GtkWidget *)box);
  gtk_widget_show( (GtkWidget *)scaleBar);
  gtk_widget_show( (GtkWidget *)drawing_area);
  gtk_widget_show( (GtkWidget *)main_window);

#ifndef NO_JOYSTICK
  if (joystick.initialized)
    gtk_timeout_add(50, (GtkFunction)Joystick_Event, NULL);
#endif
  gtk_main();
}

static void
read_robotgraph_parameters(int argc, char **argv)
{
  int num_items;

  carmen_param_t param_list[] = {
    {"robot", "rectangular",  CARMEN_PARAM_ONOFF, &rectangular, 0, NULL},
    {"robot", "length", CARMEN_PARAM_DOUBLE, &length, 0, NULL},
    {"robot", "width", CARMEN_PARAM_DOUBLE, &width, 0, NULL},
    {"robot", "max_velocity", CARMEN_PARAM_DOUBLE, &max_v, 0, NULL},
    {"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &max_phi, 0, NULL},
    {"robotgui", "connect_distance", CARMEN_PARAM_DOUBLE,
     &connect_distance, 1, NULL},
    {"robotgui", "gui_control", CARMEN_PARAM_ONOFF,
     &gui_control, 1, NULL},
    {"robotgui", "show_velocity", CARMEN_PARAM_ONOFF,
     &show_velocity, 1, NULL},
    {"robotgui", "show_vector", CARMEN_PARAM_ONOFF,
     &show_vector, 1, NULL}
  };


  num_items = sizeof(param_list)/sizeof(param_list[0]);

  carmen_param_install_params(argc, argv, param_list, num_items);

  carmen_param_check_unhandled_commandline_args(argc, argv);

  rectangular = 1;
}

int 
main(int argc, char **argv)
{  
#ifndef NO_JOYSTICK
  int err;
#endif

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

#ifndef NO_JOYSTICK
  /* initialize joystick */
  err = carmen_initialize_joystick(&joystick);
  if (err < 0)
    joystick.initialized = 0;
#endif
  read_robotgraph_parameters(argc, argv);

  if(initialize_robotgraph_ipc() < 0) 
    carmen_die("Error: could not connect to IPC Server\n");

  signal(SIGINT, shutdown_robotgraph);  

  start_graphics(argc, argv);

  return 0;
}
