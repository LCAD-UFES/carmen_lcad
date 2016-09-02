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

	glPointSize (1.0);
}

// calculates the real world position of the estimated lane positions
// and add it to a trail that will be drawn
void add_to_trail(carmen_elas_lane_analysis_message * message, lane_analysis_drawer * lane_drawer, const carmen_vector_3D_t offset) {
	// TODO: something else should be done for those cases
	if (message->num_control_points == 0) return;

	// add the points to their trails
	lane_drawer->left.push_back(sub_vectors(message->left_control_points[0], offset));
	lane_drawer->right.push_back(sub_vectors(message->right_control_points[0], offset));

	// clear the points ahead
	lane_drawer->left_ahead.clear();
	lane_drawer->right_ahead.clear();

	// put the control points into the drawer
	for (int i = 0; i < message->num_control_points; ++i) lane_drawer->left_ahead.push_back(sub_vectors(message->left_control_points[i], offset));
	for (int i = 0; i < message->num_control_points; ++i) lane_drawer->right_ahead.push_back(sub_vectors(message->right_control_points[i], offset));
}

lane_analysis_drawer* create_lane_analysis_drawer() { return new lane_analysis_drawer; }
void destroy_lane_analysis_drawer(lane_analysis_drawer * lane_drawer) { delete lane_drawer; }
