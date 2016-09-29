
#ifndef DQN_H_
#define DQN_H_

#include <map>
#include <vector>
#include "State.h"
#include "RLController.h"

#include <caffe/caffe.hpp>
#include <caffe/solver.hpp>
#include <caffe/solver_factory.hpp>
#include <caffe/sgd_solvers.hpp>
#include <caffe/util/io.hpp>
#include <caffe/layers/memory_data_layer.hpp>
#include <caffe/layers/sigmoid_cross_entropy_loss_layer.hpp>
#include <caffe/layers/euclidean_loss_layer.hpp>
#include <caffe/layers/inner_product_layer.hpp>

using namespace std;
using namespace caffe;

class QLearningController : public RLController
{
protected:

	char* _INPUT_BLOB;
	char* _TARGET_BLOB;
	char* _FILTER_BLOB;
	char* _OUTPUT_BLOB;
	char* _SOLVER_FILE;
	double FUTURE_REWARD_DISCOUNT;

	double _last_selected_command;
	double _exploration_probability;

	boost::shared_ptr<caffe::Solver<float>> _solver;
	boost::shared_ptr<caffe::Net<float>> _net;

	boost::shared_ptr<caffe::MemoryDataLayer<float>> _input;
	boost::shared_ptr<caffe::MemoryDataLayer<float>> _target;
	boost::shared_ptr<caffe::MemoryDataLayer<float>> _filter;

	vector<caffe::Datum> _build_target_datum(double command, double Q);
	vector<caffe::Datum> _build_input_datum(State state);
	vector<caffe::Datum> _build_filter_datum(int position_to_turn_on = -1);

	void _update_exploration_probability();

public:

	QLearningController(char *solver_name);

	virtual void loadTrain(char *filename);
	virtual void saveTrain();

	virtual pair<double, double> selectAction(State state, FILE *f, bool use_greedy);
	virtual void train(State state, double reward, State next_state, pair<double, double> action_and_predicted_reward, FILE *f);

};


#endif

