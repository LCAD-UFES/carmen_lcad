

#ifndef DQNINTERACTION_H_
#define DQNINTERACTION_H_


#include <deque>
#include <cfloat>
#include <cstdlib>
#include <opencv/cv.h>
#include <carmen/carmen.h>

using namespace cv;


class DqnInteration
{
	vector<float> *additional_data_vector;

	public:

		Mat* input;
		double immediate_reward;
		int action_v;
		int action_phi;
		std::deque<int> *last_commands;
		std::deque<double> *last_vs;
		std::deque<double> *last_phis;
		std::deque<carmen_point_t> *last_goal_poses;

		DqnInteration();
		DqnInteration(DqnInteration *iter);
		DqnInteration(Mat *input_p, double immediate_reward_p,
					int action_v_p, int action_phi_p, std::deque<int> *last_commands_p,
					std::deque<double> *last_vs_p, std::deque<double> *last_phis_p,
					std::deque<carmen_point_t> *last_goal_poses_p);

		void free();

		vector<float>* AdditionalDataVector();
		static void BuildDataVector(std::deque<int> *last_commands_p,
				std::deque<double> *last_vs_p, std::deque<double> *last_phis,
				std::deque<carmen_point_t> *last_goal_poses_p, vector<float>* vec);
};


#endif
