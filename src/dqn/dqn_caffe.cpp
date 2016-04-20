
#include "dqn_caffe.h"

#include <vector>
#include <caffe/caffe.hpp>
#include <caffe/solver.hpp>
#include <caffe/sgd_solvers.hpp>
#include <caffe/util/io.hpp>
#include <boost/smart_ptr.hpp>
#include <caffe/layers/memory_data_layer.hpp>
#include <glog/logging.h>


std::string action_to_string(DqnAction a)
{
	switch(a)
	{
		case DQN_ACTION_NONE:
		{
			return std::string("DQN_ACTION_NONE");
			break;
		}
		case DQN_ACTION_SPEED_DOWN:
		{
			return std::string("DQN_ACTION_SPEED_DOWN");
			break;
		}
		case DQN_ACTION_SPEED_UP:
		{
			return std::string("DQN_ACTION_SPEED_UP");
			break;
		}
		case DQN_ACTION_STEER_LEFT:
		{
			return std::string("DQN_ACTION_STEER_LEFT");
			break;
		}
		case DQN_ACTION_STEER_RIGHT:
		{
			return std::string("DQN_ACTION_STEER_RIGHT");
			break;
		}
		default:
		{
			return std::string("DQN_ACTION_INVALID");
			break;
		}
	}
}


template<typename Dtype>
bool HasBlobSize(const caffe::Blob<Dtype>& blob, const int num,
	const int channels, const int height, const int width)
{
	return ((blob.num() == num) &&
			(blob.channels() == channels) &&
			(blob.height() == height) &&
			(blob.width() == width));
}


std::string PrintQValues(const std::vector<float>& q_values, const std::vector<DqnAction>& actions)
{
	assert(!q_values.empty());
	assert(!actions.empty());
	assert(q_values.size() == actions.size());

	std::ostringstream actions_buf;
	std::ostringstream q_values_buf;

	for (size_t i = 0; i < q_values.size(); ++i)
	{
		const std::string a_str = boost::algorithm::replace_all_copy(action_to_string(actions[i]), "PLAYER_A_", "");
		const std::string q_str = boost::lexical_cast<std::string>(q_values[i]);
		const int column_size = std::max(a_str.size(), q_str.size()) + 1;

		actions_buf.width(column_size);
		actions_buf << a_str;

		q_values_buf.width(column_size);
		q_values_buf << q_str;
	}

	actions_buf << std::endl;
	q_values_buf << std::endl;

	return actions_buf.str() + q_values_buf.str();
}


DqnCaffe::DqnCaffe(DqnParams params, char *program_name)
{
	_params = params;

	if (params.USE_GPU)
	{
		caffe::Caffe::SetDevice(0);
		caffe::Caffe::set_mode(caffe::Caffe::GPU);
	}
	else
		caffe::Caffe::set_mode(caffe::Caffe::CPU);

	google::InitGoogleLogging(program_name);
	google::InstallFailureSignalHandler();
	google::LogToStderr();

	legal_actions_.push_back(DQN_ACTION_NONE);
	legal_actions_.push_back(DQN_ACTION_SPEED_DOWN);
	legal_actions_.push_back(DQN_ACTION_SPEED_UP);
	legal_actions_.push_back(DQN_ACTION_STEER_LEFT);
	legal_actions_.push_back(DQN_ACTION_STEER_RIGHT);

	replay_memory_capacity_ = params.REPLAY_MEMORY_SIZE;
	solver_param_ = params.SOLVER_FILE;
	gamma_ = params.GAMMA;
	current_iter_ = 0;

// *********************************************
// CORRIGIRRRRR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// *********************************************
//	DQN(const ActionVect& legal_actions, const std::string& solver_param,
//			const int replay_memory_capacity, const double gamma) :
//			legal_actions_(legal_actions), solver_param_(solver_param), replay_memory_capacity_(
//					replay_memory_capacity), gamma_(gamma), current_iter_(0), random_engine(
//					0)
//	{
//	}
}


void DqnCaffe::Initialize()
{
	  caffe::SolverParameter solver_param;
	  caffe::ReadProtoFromTextFileOrDie(_params.SOLVER_FILE, &solver_param);
	  //solver_ = boost::shared_ptr<caffe::AdaDeltaSolver<float> >(new caffe::AdaDeltaSolver<float>(solver_param));
	  solver_.reset(caffe::SolverRegistry<float>::CreateSolver(solver_param)); // = boost::shared_ptr<caffe::AdaDeltaSolver<float> >(new caffe::AdaDeltaSolver<float>(solver_param));
	  //solver_.reset(caffe::GetSolver<float>(solver_param)); // = boost::shared_ptr<caffe::AdaDeltaSolver<float> >(new caffe::AdaDeltaSolver<float>(solver_param));

	  net_ = solver_->net();

	  // Cache pointers to blobs that hold Q values
	  q_values_blob_ = net_->blob_by_name("q_values");

	  // Initialize dummy input data with 0
	  std::fill(dummy_input_data_.begin(), dummy_input_data_.end(), 0.0);

	  // Cache pointers to input layers
	  frames_input_layer_ = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float> >(
			  net_->layer_by_name("frames_input_layer"));

	  assert(frames_input_layer_);
	  assert(HasBlobSize(
		  *net_->blob_by_name("frames"),
		  DqnParams::kMinibatchSize,
		  DqnParams::kInputFrameCount,
		  DqnParams::kCroppedFrameSize,
		  DqnParams::kCroppedFrameSize));

	  target_input_layer_ = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float> >(net_->layer_by_name("target_input_layer"));

	  assert(target_input_layer_);
	  assert(HasBlobSize(*net_->blob_by_name("target"), DqnParams::kMinibatchSize, DqnParams::kOutputCount, 1, 1));

	  filter_input_layer_ = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float> >(net_->layer_by_name("filter_input_layer"));

	  assert(filter_input_layer_);
	  assert(HasBlobSize(*net_->blob_by_name("filter"), DqnParams::kMinibatchSize, DqnParams::kOutputCount, 1, 1));
}


void
DqnCaffe::LoadTrainedModel(const std::string& model_file)
{
	net_->CopyTrainedLayersFrom(model_file);
}


void DqnCaffe::InputDataIntoLayers(const FramesLayerInputData& frames_input,
		const TargetLayerInputData& target_input,
		const FilterLayerInputData& filter_input)
{
	frames_input_layer_->Reset(const_cast<float*>(frames_input.data()), dummy_input_data_.data(), DqnParams::kMinibatchSize);
	target_input_layer_->Reset(const_cast<float*>(target_input.data()), dummy_input_data_.data(), DqnParams::kMinibatchSize);
	filter_input_layer_->Reset(const_cast<float*>(filter_input.data()), dummy_input_data_.data(), DqnParams::kMinibatchSize);
}


std::pair<DqnAction, float> DqnCaffe::SelectActionGreedily(const InputFrames& last_frames)
{
	std::vector<InputFrames> frames_vector;
	frames_vector.push_back(last_frames);

	// *****************************
	// CHECAR SE AS DUAS INSTRUCOES ABAIXO SAO EQUIVALENTES!!
	// *****************************
	// return SelectActionGreedily(std::vector<InputFrames>{{last_frames}}).front();
	return SelectActionGreedily(frames_vector /*std::vector<InputFrames>{{last_frames}}*/).front();
}


std::vector<std::pair<DqnAction, float> > DqnCaffe::SelectActionGreedily(
		const std::vector<InputFrames>& last_frames_batch)
{
	assert(last_frames_batch.size() <= DqnParams::kMinibatchSize);

	MyArray<float, DqnParams::kMinibatchDataSize> frames_input;

	for (size_t i = 0; i < last_frames_batch.size(); ++i)
	{
		// Input frames to the net and compute Q values for each legal actions
		for (int j = 0; j < DqnParams::kInputFrameCount; ++j)
		{
			const FrameDataSp& frame_data = last_frames_batch[i][j];

			std::copy(frame_data->begin(), frame_data->end(),
				frames_input.begin() + i * DqnParams::kInputDataSize + j * DqnParams::kCroppedFrameDataSize);
		}
	}

	InputDataIntoLayers(frames_input, dummy_input_data_, dummy_input_data_);
	net_->ForwardPrefilled(NULL);

	std::vector<std::pair<DqnAction, float> > results;
	results.reserve(last_frames_batch.size());

	for (size_t i = 0; i < last_frames_batch.size(); ++i)
	{
		std::vector<float> q_values(legal_actions_.size());

		// ***********************************
		// CHECAR SE A INSTRUCAO ABAIXO DE FATO EQUIVALE AO FOR!!!!!!
		// ***********************************
		// std::transform(legal_actions_.begin(), legal_actions_.end(), q_values.begin(), action_evaluator);
		for (size_t action_id = 0; action_id < legal_actions_.size(); action_id++)
		{
			float q = q_values_blob_->data_at(i, static_cast<int>(legal_actions_[action_id]), 0, 0);
			//assert(!isnan(q));
			if (isnan(q)) q = 0;
			q_values[action_id] = q;
		}

		if (last_frames_batch.size() == 1)
		{
			std::cout << PrintQValues(q_values, legal_actions_);
		}

		// Select the action with the maximum Q value
		const int max_idx = std::distance(q_values.begin(), std::max_element(q_values.begin(), q_values.end()));
		results.push_back(std::pair<DqnAction, float>(legal_actions_[max_idx], q_values[max_idx]));
	}

	return results;
}


DqnAction
DqnCaffe::SelectAction(const InputFrames& last_frames, double epsilon)
{
	double probability;
	assert(epsilon >= 0.0 && epsilon <= 1.0);

	DqnAction action = SelectActionGreedily(last_frames).first;

	probability = (double) rand() / (double) RAND_MAX;

	if (probability < epsilon)
	{
		// Select randomly
		int random_idx = rand() % legal_actions_.size();

		action = legal_actions_[random_idx];
		std::cout << action_to_string(action) << " (random)";
	}
	else
	{
		std::cout << action_to_string(action) << " (greedy)";
	}

	std::cout << " epsilon:" << epsilon << std::endl;
	return action;
}


void
DqnCaffe::AddTransition(const Transition& transition)
{
	if (replay_memory_.size() == replay_memory_capacity_)
		replay_memory_.pop_front();

	replay_memory_.push_back(transition);
}


void
DqnCaffe::Update()
{
//	printf("\n\n**** UPDATE START\n");

	int i, j, random_transition_id;

	std::cout << "iteration: " << current_iter_++ << std::endl;

	// Sample transitions from replay memory
	std::vector<int> transitions;
	transitions.reserve(DqnParams::kMinibatchSize);

	for (i = 0; i < DqnParams::kMinibatchSize; ++i)
	{
		random_transition_id = rand() % replay_memory_.size();
		transitions.push_back(random_transition_id);
	}

	// Compute target values: max_a Q(s',a)
	std::vector<InputFrames> target_last_frames_batch;

	for (i = 0; i < (int) transitions.size(); i++)
	// for (const auto idx : transitions)
	{
		int idx = transitions[i];
		Transition& transition = replay_memory_[idx];

		if (!transition.frame_data)
		{
			// This is a terminal state
			continue;
		}

		// Compute target value
		InputFrames target_last_frames;

		for (j = 0; j < DqnParams::kInputFrameCount; ++j)
			target_last_frames.push_back(transition.input_frames[j]);

//		target_last_frames[DqnParams::kInputFrameCount - 1] = transition.frame_data; //  std::get < 3 > (transition).get();

		target_last_frames_batch.push_back(target_last_frames);
	}

	std::vector<std::pair<DqnAction, float> > actions_and_values = SelectActionGreedily(target_last_frames_batch);

	static FramesLayerInputData frames_input;
	static TargetLayerInputData target_input;
	static FilterLayerInputData filter_input;
	static int first = 1;

	if (first)
	{
		for (i = 0 ; i < frames_input.capacity(); i++)
			frames_input.push_back(0);

		for (i = 0 ; i < filter_input.capacity(); i++)
			filter_input.push_back(0);

		for (i = 0 ; i < target_input.capacity(); i++)
			target_input.push_back(0);

		first = 0;

//		std::fill(target_input.begin(), target_input.end(), 0.0f);
//		std::fill(filter_input.begin(), filter_input.end(), 0.0f);
//		std::fill(frames_input.begin(), frames_input.end(), 0.0f);
	}

	size_t target_value_id = 0;

	for (i = 0; i < DqnParams::kMinibatchSize; ++i)
	{
		Transition& transition = replay_memory_[transitions[i]];
		DqnAction action = transition.action; //std::get < 1 > (transition);

		assert(static_cast<int>(action) < DqnParams::kOutputCount);

		double reward = transition.reward; // std::get < 2 > (transition);

		// ***********************************
		// WHY IS THIS IMPORTANT????
		// ***********************************
		//assert(reward >= -1.0 && reward <= 1.0);

		double target = 0.0;

		if (transition.frame_data)
			target = reward + gamma_ * actions_and_values[target_value_id++].second;
		else
			target = reward;

		assert(!isnan(target));

		target_input[i * DqnParams::kOutputCount + static_cast<int>(action)] = target;
		filter_input[i * DqnParams::kOutputCount + static_cast<int>(action)] = 1;

		VLOG(1) << "filter:" << action_to_string(action) << " target:" << target;

		for (j = 0; j < DqnParams::kInputFrameCount; ++j)
		{
			FrameDataSp& frame_data = transition.input_frames[j]; //std::get < 0 > (transition)[j];

			std::copy(frame_data->begin(), frame_data->end(),
				frames_input.begin() + i * DqnParams::kInputDataSize + j * DqnParams::kCroppedFrameDataSize);
		}
	}

	InputDataIntoLayers(frames_input, target_input, filter_input);
	solver_->Step(1);

	// Log the first parameter of each hidden layer
	VLOG(1) << "conv1:" << net_->layer_by_name("conv1_layer")->blobs().front()->data_at(1, 0, 0, 0);
	VLOG(1) << "conv2:" << net_->layer_by_name("conv2_layer")->blobs().front()->data_at(1, 0, 0, 0);
	VLOG(1) << "ip1:" << net_->layer_by_name("ip1_layer")->blobs().front()->data_at(1, 0, 0, 0);
	VLOG(1)	<< "ip2:" << net_->layer_by_name("ip2_layer")->blobs().front()->data_at(1, 0, 0, 0);

//	printf("**** UPDATE END\n\n");
}


