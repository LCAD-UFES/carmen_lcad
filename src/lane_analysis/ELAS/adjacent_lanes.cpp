#include "adjacent_lanes.h"

using namespace std;
using namespace cv;

// vars
static vector<int> adj_left_buffer, adj_right_buffer;
static int adj_left_previous = -1, adj_right_previous = -1;

void ELAS::adjacent_lanes_detection(const feature_maps * _feature_maps, const lane_position * _lane_position, const raw_houghs * _raw_houghs, const lane_marking_type * _lmt, ConfigXML * _cfg, adjacent_lanes * out) {
	printf("adjacent_lanes_detection()\n");

	vector<int> adj_lanes = faixasAdjacentesHoughs(_feature_maps->map_srf_ipm, _raw_houghs->ego_lane, _raw_houghs->adjacent_lanes, _lane_position->lane_base, (LMT)_lmt->left, (LMT)_lmt->right, _cfg);

	// assume the current (estimated) is the correct one
	out->left = adj_lanes[0];
	out->right = adj_lanes[1];

	// TODO: create a function to the operations below (hysteresis)

	// rejects temporary transitions (hysteresis)
	if (adj_left_previous == -1) adj_left_previous = out->left;									// first time
	else if (out->left == adj_left_previous) adj_left_buffer.clear();							// the current is equal to the previous, ensure an empty buffer
	else if (out->left != adj_left_previous) {													// if the current is different from the previous one
		if (adj_left_buffer.size() == ADJACENT_LANES_BUFFER_SIZE) adj_left_buffer.clear();		//   if the buffer is full
		else {																					// 	 else,
			adj_left_buffer.push_back(out->left);												//	   add the current to the buffer
			out->left = adj_left_previous;														//	   and defines the correct one is the previous
		}
	}

	if (adj_right_previous == -1) adj_right_previous = out->right;
	else if (out->right == adj_right_previous) adj_right_buffer.clear();
	else if (out->right != adj_right_previous) {
		if (adj_right_buffer.size() == ADJACENT_LANES_BUFFER_SIZE) adj_right_buffer.clear();
		else {
			adj_right_buffer.push_back(out->right);
			out->right = adj_right_previous;
		}
	}

	adj_left_previous = out->left;
	adj_right_previous = out->right;
}
