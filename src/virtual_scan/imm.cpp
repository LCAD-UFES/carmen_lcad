#include <carmen/carmen.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/velodyne_interface.h>
#include <carmen/matrix.h>
#include "virtual_scan.h"


void
fit_multiple_models_to_track_of_hypotheses(virtual_scan_track_t *track)
{
	Matrix Tp,Tc;
}


void
update_hypotheses_state(virtual_scan_track_t *track)
{
	if (track == NULL)
		return;

	if (track->size >= 2)
	{
		for (int j = 1; j < track->size; j++)
		{
			double delta_t = track->box_model_hypothesis[j].hypothesis_points.precise_timestamp - track->box_model_hypothesis[j - 1].hypothesis_points.precise_timestamp;
			double distance_travelled = DIST2D(track->box_model_hypothesis[j].hypothesis, track->box_model_hypothesis[j - 1].hypothesis);
			double angle_in_the_distance_travelled = ANGLE2D(track->box_model_hypothesis[j].hypothesis, track->box_model_hypothesis[j - 1].hypothesis);
			double v = (cos(track->box_model_hypothesis[j].hypothesis.theta - angle_in_the_distance_travelled) * distance_travelled) / delta_t;
			double delta_theta = carmen_normalize_theta(track->box_model_hypothesis[j].hypothesis.theta - track->box_model_hypothesis[j - 1].hypothesis.theta);
			double d_theta = delta_theta / delta_t;
			track->box_model_hypothesis[j].hypothesis_state.v = v;
			track->box_model_hypothesis[j].hypothesis_state.d_theta = d_theta;
		}
	}

	fit_multiple_models_to_track_of_hypotheses(track);
}
