
#include <carmen/carmen.h>
#include "QLearningController.h"
#include "Util.h"


vector<caffe::Datum>
QLearningController::_build_input_datum(State state)
{
	int i;
	caffe::Datum d;
	vector<caffe::Datum> datums;

	int input_size = _NUM_FUTURE_VELOCITIES + _NUM_PAST_VELOCITIES + _NUM_PAST_COMMANDS;

	d.set_channels(1);
	d.set_height(1);
	d.set_width(input_size);
	d.set_label(0);

	//printf("state.desired_v.size(): %ld\n", state.desired.size());
	//printf("state.measured_v.size(): %ld\n", state.measured.size());
	//printf("state.last_commmands.size(): %ld\n", state.last_commmands.size());

	//for (i = 0; i < state.desired.size(); i++)
	//	printf("state.desired[%d]: %lf\n", i, state.desired[i] / 60.0);
    //
	//for (i = 0; i < state.measured.size(); i++)
	//	printf("state.measured[%d]: %lf\n", i, state.measured[i] / 60.0);
    //
	//for (i = 0; i < state.last_commmands.size(); i++)
	//	printf("state.last_commmands[%d]: %lf\n", i, state.last_commmands[i] / 100.0);

	//exit(0);

	for (i = 0; i < _NUM_FUTURE_VELOCITIES; i++)
	{
		if (i >= state.desired.size())
			d.add_float_data(0);
		else
			d.add_float_data(state.desired[i] / 60.0);
	}

	for (i = 0; i < _NUM_PAST_VELOCITIES; i++)
	{
		if (i >= state.measured.size())
			d.add_float_data(0);
		else
			d.add_float_data(state.measured[i] / 60.0);
	}

	for (i = 0; i < _NUM_PAST_COMMANDS; i++)
	{
		if (i >= state.last_commmands.size())
			d.add_float_data(0);
		else
			d.add_float_data(state.last_commmands[i] / 100.0);
	}

	datums.push_back(d);
	return datums;
}


vector<caffe::Datum>
QLearningController::_build_target_datum(double command, double Q)
{
	caffe::Datum d;
	vector<caffe::Datum> datums;

	d.set_channels(1);
	d.set_height(1);
	d.set_width(_net->blob_by_name(_OUTPUT_BLOB)->channels());
	d.set_label(0);

	for (int i = 0; i < _net->blob_by_name(_OUTPUT_BLOB)->channels(); i++)
	{
		if (i == command)
			d.add_float_data(Q);
		else
			d.add_float_data(0.0);
	}

//	printf("TARGET: ");
//	for (int i = 0; i < _net->blob_by_name(OUTPUT_BLOB)->channels(); i++)
//		printf("%.2lf ", d.float_data(i));
//	printf("\n");

	datums.push_back(d);
	return datums;
}


vector<caffe::Datum>
QLearningController::_build_filter_datum(int position_to_turn_on)
{
	caffe::Datum d;
	vector<caffe::Datum> datums;

	d.set_channels(1);
	d.set_height(1);
	d.set_width(_net->blob_by_name(_OUTPUT_BLOB)->channels());
	d.set_label(0);

	for (int i = 0; i < _net->blob_by_name(_OUTPUT_BLOB)->channels(); i++)
	{
		if (i == position_to_turn_on)
			d.add_float_data(1.0);
		else
			d.add_float_data(0.0);
	}

//	printf("FILTER: ");
//	for (int i = 0; i < _net->blob_by_name(OUTPUT_BLOB)->channels(); i++)
//		printf("%.2lf ", d.float_data(i));
//	printf("\n");

	datums.push_back(d);
	return datums;
}


QLearningController::QLearningController(char *solver_name) : RLController()
{
	_INPUT_BLOB = "input";
	_TARGET_BLOB = "target";
	_FILTER_BLOB = "filter";
	_OUTPUT_BLOB = "fc3";
	_SOLVER_FILE = solver_name;
	FUTURE_REWARD_DISCOUNT = 0.9;

	_last_selected_command = 0;

	caffe::SolverParameter solver_param;

	caffe::ReadProtoFromTextFileOrDie(_SOLVER_FILE, &solver_param);
	_solver.reset(caffe::SolverRegistry<float>::CreateSolver(solver_param));
	_net = _solver->net();

	_input = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(_net->layer_by_name(_INPUT_BLOB));
	_target = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(_net->layer_by_name(_TARGET_BLOB));
	_filter = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(_net->layer_by_name(_FILTER_BLOB));

	_exploration_probability = 1.0;
}


void
QLearningController::_update_exploration_probability()
{
	_exploration_probability += (0.1 - 1.0) / (100000.0); // 20 files

	if (_exploration_probability < 0.1)
		_exploration_probability = 0.1;
}


pair<double, double>
QLearningController::selectAction(State state, FILE *f, bool use_greedy = false)
{
//	static int i = 0;
//
//	i++;
//
//	if (i == 10)
//		return pair<double, double>(1,1);
//	else if (i == 20)
//		return pair<double, double>(-1,1);
//	else if (i >= 80)
//		exit(0);
//	else
//		return pair<double, double>(0,0);

	vector<caffe::Datum> input_datum = _build_input_datum(state);
	vector<caffe::Datum> dummy_target_datum = _build_target_datum(0, 0);
	vector<caffe::Datum> dummy_filter_datum = _build_filter_datum();

	_input->AddDatumVector(input_datum);
	_target->AddDatumVector(dummy_target_datum);
	_filter->AddDatumVector(dummy_filter_datum);
	_net->ForwardPrefilled(NULL);

	int maxid = 0;
	double maxreward = -DBL_MAX;

	for (int i = 0; i < _net->blob_by_name(_OUTPUT_BLOB)->channels(); i++)
	{
		if (_net->blob_by_name(_OUTPUT_BLOB)->cpu_data()[i] > maxreward)
		{
			maxreward = _net->blob_by_name(_OUTPUT_BLOB)->cpu_data()[i];
			maxid = i;
		}
	}

	if (!use_greedy)
	{
		_update_exploration_probability();

		if (f != NULL)
		{
//			fprintf(f, "Explo: %.3lf ", _exploration_probability);
//			fflush(f);
		}

		if (unitary_rand() < _exploration_probability)
		{
			if (_exploration_probability <= 0.5)
			{
				// weak random exploration
				if (rand() % 2 == 0) maxid = maxid + 1;
				else maxid = maxid - 1;

				if (maxid > _net->blob_by_name(_OUTPUT_BLOB)->channels())
					maxid = _net->blob_by_name(_OUTPUT_BLOB)->channels();

				if (maxid < 0)
					maxid = 0;

				maxreward = _net->blob_by_name(_OUTPUT_BLOB)->cpu_data()[maxid];
			}
			else
			{
				// strong random exploration
				maxid = rand() % _net->blob_by_name(_OUTPUT_BLOB)->channels();
			}
		}
	}
	else // just to make the output uniform
	{
		if (f != NULL)
		{
//			fprintf(f, "Explo: %.3lf ", 0.0);
//			fflush(f);
		}
	}

	//_last_selected_command = maxid;
	// double command = ((double) maxid / (double) (_net->blob_by_name(_OUTPUT_BLOB)->channels() - 1)) * (_MAX_CONTROL - _MIN_CONTROL) + _MIN_CONTROL;
	return pair<double, double>(maxid, maxreward);
}


void
QLearningController::train(State state, double reward, State next_state, pair<double, double> action_and_predicted_reward, FILE *f)
{
	vector<caffe::Datum> input_next_state_datum = _build_input_datum(next_state);
	vector<caffe::Datum> dummy_target_datum = _build_target_datum(0, 0);
	vector<caffe::Datum> dummy_filter_datum = _build_filter_datum();

	_input->AddDatumVector(input_next_state_datum);
	_target->AddDatumVector(dummy_target_datum);
	_filter->AddDatumVector(dummy_filter_datum);

	_net->ForwardPrefilled(NULL);

	double max_future_reward = -DBL_MAX;

	for (int i = 0; i < _net->blob_by_name(_OUTPUT_BLOB)->channels(); i++)
		if (_net->blob_by_name(_OUTPUT_BLOB)->cpu_data()[i] > max_future_reward)
			max_future_reward = _net->blob_by_name(_OUTPUT_BLOB)->cpu_data()[i];

	double action_value = FUTURE_REWARD_DISCOUNT * max_future_reward + reward;

	if (f != NULL)
	{
//		fprintf(f, "TD: %lf ", fabs(action_value - action_and_predicted_reward.second));
//		fflush(f);
	}

	vector<caffe::Datum> input_datum = _build_input_datum(state);
	vector<caffe::Datum> target_datum = _build_target_datum(action_and_predicted_reward.first, action_value);
	vector<caffe::Datum> filter_datum = _build_filter_datum(action_and_predicted_reward.first);

	_input->AddDatumVector(input_datum);
	_target->AddDatumVector(target_datum);
	_filter->AddDatumVector(filter_datum);
	_solver->Step(5);
}


void
QLearningController::loadTrain(char *filename)
{
	_net->CopyTrainedLayersFrom(filename);
}


void
QLearningController::saveTrain()
{
	_solver->Snapshot();
}


