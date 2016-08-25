#include "lane_analysis_drawer.h"
#include "math.h"

void draw_lane_analysis(lane_analysis_drawer * lane_drawer) {
	bool draw_lines = true;
	glPointSize (3.0);

		// TRAIL
	    // ==============================================================================================================

	    // left
	    for (unsigned int i = 0; i < lane_drawer->left.size(); i++) {

	    	glBegin (GL_POINTS);
				glColor3d (0.0, 1.0, 1.0);
				glVertex3d (lane_drawer->left[i].x, lane_drawer->left[i].y, lane_drawer->left[i].z);
	        glEnd ();

	        if (draw_lines && i > 0) {
				glBegin(GL_LINES);
					glVertex3d (lane_drawer->left[i].x, lane_drawer->left[i].y, lane_drawer->left[i].z);
					glVertex3d (lane_drawer->left[i-1].x, lane_drawer->left[i-1].y, lane_drawer->left[i-1].z);
				glEnd();
	        }
	    }

	    // right
	    for (unsigned int i = 0; i < lane_drawer->right.size(); i++) {

	    	glBegin (GL_POINTS);
				glColor3d (0.0, 1.0, 1.0);
				glVertex3d (lane_drawer->right[i].x, lane_drawer->right[i].y, lane_drawer->right[i].z);
			glEnd ();

			if (draw_lines && i > 0) {
				glBegin(GL_LINES);
					glVertex3d (lane_drawer->right[i].x, lane_drawer->right[i].y, lane_drawer->right[i].z);
					glVertex3d (lane_drawer->right[i-1].x, lane_drawer->right[i-1].y, lane_drawer->right[i-1].z);
				glEnd();
			}
		}

	    // LANE AHEAD
	    // ==============================================================================================================

	    // left
	    for (unsigned int i = 0; i < lane_drawer->left_ahead.size(); i++) {
	    	glBegin (GL_POINTS);
				glColor3d (1.0, 1.0, 0.0);
				glVertex3d (lane_drawer->left_ahead[i].x, lane_drawer->left_ahead[i].y, lane_drawer->left_ahead[i].z);
			glEnd ();

			if (draw_lines && i > 0) {
				glBegin(GL_LINES);
					glVertex3d (lane_drawer->left_ahead[i].x, lane_drawer->left_ahead[i].y, lane_drawer->left_ahead[i].z);
					glVertex3d (lane_drawer->left_ahead[i-1].x, lane_drawer->left_ahead[i-1].y, lane_drawer->left_ahead[i-1].z);
				glEnd();
			}
	    }

	    // right
		for (unsigned int i = 0; i < lane_drawer->right_ahead.size(); i++) {
			glBegin (GL_POINTS);
				glColor3d (1.0, 1.0, 0.0);
				glVertex3d (lane_drawer->right_ahead[i].x, lane_drawer->right_ahead[i].y, lane_drawer->right_ahead[i].z);
			glEnd ();

			if (draw_lines && i > 0) {
				glBegin(GL_LINES);
					glVertex3d (lane_drawer->right_ahead[i].x, lane_drawer->right_ahead[i].y, lane_drawer->right_ahead[i].z);
					glVertex3d (lane_drawer->right_ahead[i-1].x, lane_drawer->right_ahead[i-1].y, lane_drawer->right_ahead[i-1].z);
				glEnd();
			}
		}

		glBegin(GL_LINES);
			glVertex3d (elas_direction.x, elas_direction.y, elas_direction.z);
			glVertex3d (elas_direction.x*10, elas_direction.y*10, elas_direction.z);
		glEnd();

	glPointSize (1.0);
}

// calculates the real world position of the estimated lane positions
// and add it to a trail that will be drawn
void add_to_trail(carmen_elas_lane_estimation_message * message, const carmen_pose_3D_t &car_pose, lane_analysis_drawer * lane_drawer, carmen_vector_3D_t * localize_ackerman_trail, int localize_ackerman_size) {

	// TODO: break line sequence in these cases, to show the discontinuity
	// if the message contains NaN, do not add to the trail
	if (std::isnan(message->lane_width)) return;

	const double _scale_y = 0.25; //message->scale_y;
	const double lane_width = message->lane_width * message->scale_x;
	const double deviation = message->lane_deviation;

	// set the distance from the pose to the visible region on the camera
	const int shift = 0.1;

	// calculates the distance of the left lane markings from the car center
	const double dist_left = (1 + deviation) * (lane_width / 2.0);
	const double dist_right = (1 - deviation) * (lane_width / 2.0);

	// car_pose.yaw -> localize_ack_globalpos.theta
	const double _theta = car_pose.orientation.yaw;

	// calculates the unit vector of the car orientation
	carmen_vector_3D_t car_pos = car_pose.position;
	cv::Point2d _orientation_unit;
	_orientation_unit.x = cos(_theta);
	_orientation_unit.y = sin(_theta);
	_orientation_unit *= 1.0/cv::norm(_orientation_unit); // convert to unit vector

	// calculates the unit vector orthogonal to the car orientation, to draw lane position
	carmen_vector_2D_t _orthogonal_unit;
	_orthogonal_unit.x = -1 * _orientation_unit.y;
	_orthogonal_unit.y = _orientation_unit.x;

	// left point
	carmen_vector_3D_t _left;
	_left.x = car_pos.x + shift * _orientation_unit.x + dist_left * _orthogonal_unit.x;
	_left.y = car_pos.y + shift * _orientation_unit.y + dist_left * _orthogonal_unit.y;
	_left.z = car_pos.z;

	// right point
	carmen_vector_3D_t _right;
	_right.x = car_pos.x + shift * _orientation_unit.x - dist_right * _orthogonal_unit.x;
	_right.y = car_pos.y + shift * _orientation_unit.y - dist_right * _orthogonal_unit.y;
	_right.z = car_pos.z;

	// add the points to their trails
	lane_drawer->left.push_back(_left);
	lane_drawer->right.push_back(_right);

	// clear the points ahead
	lane_drawer->left_ahead.clear();
	lane_drawer->right_ahead.clear();

	// calculate the points ahead
	// they will be calculated based on their relative distances to the bottom points

	// first point is the bottom one
	lane_drawer->left_ahead.push_back(_left); // first point is the bottom one
	for (int i = 1; i < message->num_outputs_left; i++) {
		// calculates the difference to the base point in pixels
		carmen_vector_2D_t _diff;
		_diff.x = message->left_ipm[0].x - message->left_ipm[i].x;
		_diff.y = message->left_ipm[0].y - message->left_ipm[i].y;

		// convert to meters
		_diff.x *= message->scale_x;
		_diff.y *= _scale_y; // message->scale_y;

		// calculates the point position given the relative distance
		carmen_vector_3D_t _point;
		_point.x = lane_drawer->left_ahead[0].x + _diff.y * _orientation_unit.x + _diff.x * _orthogonal_unit.x;
		_point.y = lane_drawer->left_ahead[0].y + _diff.y * _orientation_unit.y + _diff.x * _orthogonal_unit.y;
		_point.z = lane_drawer->left_ahead[0].z;

		// add the point to the vector
		lane_drawer->left_ahead.push_back(_point);
	}

	// first point is the bottom one
	lane_drawer->right_ahead.push_back(_right);
	for (int i = 1; i < message->num_outputs_right; i++) {
		// calculates the difference to the base point in pixels
		carmen_vector_2D_t _diff;
		_diff.x = message->right_ipm[0].x - message->right_ipm[i].x;
		_diff.y = message->right_ipm[0].y - message->right_ipm[i].y;

		// convert to meters
		_diff.x *= message->scale_x;
		_diff.y *= _scale_y; // message->scale_y;

		// calculates the point position given the relative distance
		carmen_vector_3D_t _point;
		_point.x = lane_drawer->right_ahead[0].x + _diff.y * _orientation_unit.x + _diff.x * _orthogonal_unit.x;
		_point.y = lane_drawer->right_ahead[0].y + _diff.y * _orientation_unit.y + _diff.x * _orthogonal_unit.y;
		_point.z = lane_drawer->right_ahead[0].z;

		// add the point to the vector
		lane_drawer->right_ahead.push_back(_point);
	}
}

lane_analysis_drawer* create_lane_analysis_drawer() { return new lane_analysis_drawer; }
void destroy_lane_analysis_drawer(lane_analysis_drawer * lane_drawer) { delete lane_drawer; }
