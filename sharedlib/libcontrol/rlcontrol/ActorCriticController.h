
#ifndef _ACTOR_CRITIC_CONTROLLER_H_
#define _ACTOR_CRITIC_CONTROLLER_H_

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

class ActorCriticController : public RLController
{
protected:

	char* _INPUT_BLOB;
	char* _TARGET_BLOB;
	char* _OUTPUT_BLOB;
	char* _SOLVER_FILE;
	double _FUTURE_REWARD_DISCOUNT;
	double _EXPLORATION_COEFFICIENT;

	double _last_selected_command;
	double _exploration_probability;

	boost::shared_ptr<caffe::Solver<float>> _solver;
	boost::shared_ptr<caffe::Net<float>> _net;

	boost::shared_ptr<caffe::MemoryDataLayer<float>> _input;
	boost::shared_ptr<caffe::MemoryDataLayer<float>> _target;

	vector<caffe::Datum> _build_target_datum(double command, double value);
	vector<caffe::Datum> _build_input_datum(State state);

	void _update_exploration_probability();

public:

	ActorCriticController(char *solver_name);

	virtual void loadTrain(char *filename);
	virtual void saveTrain();

	virtual pair<double, double> selectAction(State state, FILE *f, bool use_greedy);
	virtual void train(State state, double reward, State next_state, pair<double, double> action_and_predicted_reward, FILE *f);
};


#endif

