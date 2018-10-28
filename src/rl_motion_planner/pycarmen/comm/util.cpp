
#include <cmath>
#include <carmen/carmen.h>
#include <carmen/laser_interface.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include "g2o/types/slam2d/se2.h"


std::vector<double>
laser_to_vec(carmen_laser_laser_message &laser_message, double theta, int view)
{
	int i;
	std::vector<double> readings;

	for (i = 0; i < laser_message.num_readings; i++)
		readings.push_back(laser_message.range[i]);

    // Uncomment the following lines to enable the laser viewer. I left it commented to 
    // avoid having opencv as a dependency of the module. The following two are just to
    // pretend we're using the variables so the compiler does not send warnings.
    // (void) view;
    (void) theta;

	if (view)
	{
		int px, py;
		double x, y;
		double a;
		cv::Mat m(500, 500, CV_8UC3);

		memset(m.data, (uchar) 255, m.rows * m.cols * 3 * sizeof(uchar));
		a = laser_message.config.start_angle;

		cv::circle(m, cv::Point(m.cols / 2, m.rows / 2), 2, cv::Scalar(0, 0, 255), -1);

		for (i = 0; i < laser_message.num_readings; i++)
		{
			a += laser_message.config.angular_resolution;

			x = cos(a) * laser_message.range[i];
			y = sin(a) * laser_message.range[i];

			px = x * 5 + m.cols / 2.0;
			py = m.rows - (y * 5 + m.rows / 2.0);

			cv::circle(m, cv::Point(px, py), 2, cv::Scalar(0, 0, 0), -1);
		}

		cv::imshow("laser", m);
		cv::waitKey(1);
	}

	return readings;
}


std::vector<double>
next_goal_from_list(double x, double y, double th,
		carmen_behavior_selector_goal_list_message *goal_list_message)
{
	g2o::SE2 pose(x, y, th);

	for (int i = 0; i < goal_list_message->size; i++)
	{
		g2o::SE2 goal(goal_list_message->goal_list[i].x,
			goal_list_message->goal_list[i].y,
			goal_list_message->goal_list[i].theta);

		goal = pose.inverse() * goal;

		// return the index of the first goal that is in front of the car.
		if (goal[0] > 0.)
		{
			std::vector<double> goal;

			goal.push_back(goal_list_message->goal_list[i].x);
			goal.push_back(goal_list_message->goal_list[i].y);
			goal.push_back(goal_list_message->goal_list[i].theta);
			goal.push_back(goal_list_message->goal_list[i].v);
			goal.push_back(goal_list_message->goal_list[i].phi);

			return goal;
		}
	}

	return std::vector<double>();
}


void
ackerman_motion_model(double *x, double *y, double *th, double v, double phi, double dt,
		double distance_between_front_and_rear_axles)
{
	(*x) +=  v * dt * cos(*th);
	(*y) +=  v * dt * sin(*th);
	(*th) += v * dt * tan(phi) / distance_between_front_and_rear_axles;
	(*th) = carmen_normalize_theta(*th);
}


