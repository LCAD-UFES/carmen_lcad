

#include <carmen/carmen.h>
#include "ActorCriticController.h"
#include "Util.h"


vector<caffe::Datum>
ActorCriticController::_build_input_datum(State state)
{
	int i;
	caffe::Datum d;
	vector<caffe::Datum> datums;

	int input_size = _NUM_FUTURE_VELOCITIES + _NUM_PAST_VELOCITIES + _NUM_PAST_COMMANDS;

	d.set_channels(1);
	d.set_height(1);
	d.set_width(input_size);
	d.set_label(0);

//	printf("state.desired_v.size(): %ld\n", state.desired_v.size());
//	printf("state.measured_v.size(): %ld\n", state.measured_v.size());
//	printf("state.last_commmands.size(): %ld\n", state.last_commmands.size());
//	exit(0);

	for (i = 0; i < _NUM_FUTURE_VELOCITIES; i++)
	{
		if (i >= state.desired.size())
			d.add_float_data(0);
		else
			d.add_float_data(state.desired[i] / _MAX_CONTROL);
	}

	for (i = 0; i < _NUM_PAST_VELOCITIES; i++)
	{
		if (i >= state.measured.size())
			d.add_float_data(0);
		else
			d.add_float_data(state.measured[i] / _MAX_CONTROL);
	}

	for (i = 0; i < _NUM_PAST_COMMANDS; i++)
	{
		if (i >= state.last_commmands.size())
			d.add_float_data(0);
		else
			d.add_float_data(state.last_commmands[i]);
	}

	datums.push_back(d);
	return datums;
}


vector<caffe::Datum>
ActorCriticController::_build_target_datum(double command, double value)
{
	caffe::Datum d;
	vector<caffe::Datum> datums;

	d.set_channels(1);
	d.set_height(1);
	d.set_width(2);
	d.set_label(0);

	d.add_float_data(command);
	d.add_float_data(value);

	datums.push_back(d);
	return datums;
}


ActorCriticController::ActorCriticController(char *solver_name) : RLController()
{
	_INPUT_BLOB = "input";
	_TARGET_BLOB = "target";
	_OUTPUT_BLOB = "fc3";
	_SOLVER_FILE = solver_name;
	_FUTURE_REWARD_DISCOUNT = 0.9;
	_EXPLORATION_COEFFICIENT = 0.05;

	_last_selected_command = 0;

	caffe::SolverParameter solver_param;

	caffe::ReadProtoFromTextFileOrDie(_SOLVER_FILE, &solver_param);
	_solver.reset(caffe::SolverRegistry<float>::CreateSolver(solver_param));
	_net = _solver->net();

	_input = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(_net->layer_by_name(_INPUT_BLOB));
	_target = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(_net->layer_by_name(_TARGET_BLOB));

	_exploration_probability = 1.0;
}


void
ActorCriticController::_update_exploration_probability()
{
	_exploration_probability += (0.1 - 1.0) / (100000.0);

	if (_exploration_probability < 0.1)
		_exploration_probability = 0.1;
}


pair<double, double>
ActorCriticController::selectAction(State state, FILE *f, bool use_greedy = false)
{
	vector<caffe::Datum> input_datum = _build_input_datum(state);
	vector<caffe::Datum> dummy_target_datum = _build_target_datum(0, 0);

	_input->AddDatumVector(input_datum);
	_target->AddDatumVector(dummy_target_datum);
	_net->ForwardPrefilled(NULL);

	double command = _net->blob_by_name(_OUTPUT_BLOB)->cpu_data()[0];
	double predicted_reward = _net->blob_by_name(_OUTPUT_BLOB)->cpu_data()[1];

	if (!use_greedy)
	{
		_update_exploration_probability();

		if (f != NULL)
		{
//			fprintf(f, "Explo: %.3lf ", _exploration_probability);
//			fflush(f);
		}

		if (carmen_double_random(1.0) < _exploration_probability)
			command += carmen_normal_distribution(_EXPLORATION_COEFFICIENT * (_MAX_CONTROL - _MIN_CONTROL));
	}
	else // just to make the output uniform
	{
		if (f != NULL)
		{
			fprintf(f, "Explo: %.3lf ", 0.0);
			fflush(f);
		}
	}

	if (command > _MAX_CONTROL) command = 1.0;
	if (command < _MIN_CONTROL) command = -1.0;

	_last_selected_command = command;
	return pair<double, double>(command, predicted_reward);
}


void
ActorCriticController::train(State state, double reward, State next_state, pair<double, double> action_and_predicted_reward, FILE *f)
{
	vector<caffe::Datum> input_next_state_datum = _build_input_datum(next_state);
	vector<caffe::Datum> dummy_target_datum = _build_target_datum(0, 0);

	_input->AddDatumVector(input_next_state_datum);
	_target->AddDatumVector(dummy_target_datum);
	_net->ForwardPrefilled(NULL);

	double future_value_estimation = _net->blob_by_name(_OUTPUT_BLOB)->cpu_data()[1];
	double action_value = _FUTURE_REWARD_DISCOUNT * future_value_estimation + reward;

	if (f != NULL)
	{
		fprintf(f, "TD: %lf ", fabs(action_value - action_and_predicted_reward.second));
		fflush(f);
	}

	vector<caffe::Datum> input_datum = _build_input_datum(state);
	vector<caffe::Datum> target_datum = _build_target_datum(action_and_predicted_reward.first, action_value);

	_input->AddDatumVector(input_datum);
	_target->AddDatumVector(target_datum);
	_solver->Step(5);
}


void
ActorCriticController::loadTrain(char *filename)
{
	_net->CopyTrainedLayersFrom(filename);
}


void
ActorCriticController::saveTrain()
{
	_solver->Snapshot();
}

