
#include "dqn_caffe.h"

#include <vector>
#include <iostream>
#include <caffe/caffe.hpp>
#include <caffe/solver.hpp>
#include <caffe/sgd_solvers.hpp>
#include <caffe/util/io.hpp>
#include <boost/smart_ptr.hpp>
#include <caffe/layers/memory_data_layer.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

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

	odometry_input_layer_ = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float> >(net_->layer_by_name("odometry_input_layer"));

	assert(odometry_input_layer_);
	assert(HasBlobSize(*net_->blob_by_name("odometry"), DqnParams::kMinibatchSize, 2, 1, 1));
}


void
DqnCaffe::LoadTrainedModel(const std::string& model_file)
{
	net_->CopyTrainedLayersFrom(model_file);
}


void DqnCaffe::InputDataIntoLayers(const FramesLayerInputData &frames_input,
		const TargetLayerInputData &target_input,
		const FilterLayerInputData &filter_input,
		const OdometryLayerInputData &odometry_input)
{
	frames_input_layer_->Reset(const_cast<float*>(frames_input.data()), dummy_input_data_.data(), DqnParams::kMinibatchSize);
	target_input_layer_->Reset(const_cast<float*>(target_input.data()), dummy_input_data_.data(), DqnParams::kMinibatchSize);
	filter_input_layer_->Reset(const_cast<float*>(filter_input.data()), dummy_input_data_.data(), DqnParams::kMinibatchSize);
	odometry_input_layer_->Reset(const_cast<float*>(odometry_input.data()), dummy_input_data_.data(), DqnParams::kMinibatchSize);
}


std::pair<DqnAction, float>
DqnCaffe::SelectActionGreedily(const InputFrames& last_frames, double v, double phi, std::vector<std::vector<float> > *qs /* default = NULL */)
{
	std::vector<InputFrames> frames_vector;
	std::vector<float> v_vector, phi_vector;

	frames_vector.push_back(last_frames);

	v_vector.push_back(v);
	phi_vector.push_back(phi);

	// *****************************
	// CHECAR SE AS DUAS INSTRUCOES "std::vector<InputFrames>{{last_frames}}" e "frames_vector.push_back(last_frames);" SAO DE FATO EQUIVALENTES!!
	// *****************************
	return SelectActionGreedily(frames_vector, v_vector, phi_vector, qs).front();
}


std::vector<std::pair<DqnAction, float> > DqnCaffe::SelectActionGreedily(
		const std::vector<InputFrames>& last_frames_batch, const std::vector<float>& v_batch,
		const std::vector<float>& phi_batch, std::vector<std::vector<float> > *qs /* default = NULL */)
{
	assert(last_frames_batch.size() <= DqnParams::kMinibatchSize);

	MyArray<float, DqnParams::kMinibatchDataSize> frames_input;
	OdometryLayerInputData odometry;

	for (size_t i = 0; i < DqnParams::kMinibatchSize; ++i)
	{
		// Input frames to the net and compute Q values for each legal actions
		for (int j = 0; j < DqnParams::kInputFrameCount; ++j)
		{
			if (i < last_frames_batch.size())
			{
				const FrameDataSp& frame_data = last_frames_batch[i][j];

				std::copy(frame_data->begin(), frame_data->end(),
					frames_input.begin() + i * DqnParams::kInputDataSize + j * DqnParams::kCroppedFrameDataSize);
			}
			else
			{
				for (int k = 0; k < DqnParams::kCroppedFrameDataSize; k++)
					frames_input[i * DqnParams::kInputDataSize + j * DqnParams::kCroppedFrameDataSize + k] = 0;
			}
		}
	}

	for (size_t i = 0; i < DqnParams::kMinibatchSize; ++i)
	{
		if (i < v_batch.size())
		{
			odometry.push_back(v_batch[i]);
			odometry.push_back(phi_batch[i]);
		}
		else
		{
			odometry.push_back(0);
			odometry.push_back(0);
		}
	}

	InputDataIntoLayers(frames_input, dummy_input_data_, dummy_input_data_, odometry);
	net_->ForwardPrefilled(NULL);

	std::vector<std::pair<DqnAction, float> > results;
	results.reserve(last_frames_batch.size());

	if (qs != NULL) qs->clear();

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

		if (qs != NULL) qs->push_back(q_values);

//		if (last_frames_batch.size() == 1)
//		{
//			std::cout << PrintQValues(q_values, legal_actions_);
//		}

		// Select the action with the maximum Q value
		const int max_idx = std::distance(q_values.begin(), std::max_element(q_values.begin(), q_values.end()));
		results.push_back(std::pair<DqnAction, float>(legal_actions_[max_idx], q_values[max_idx]));
	}

	return results;
}


std::pair<DqnAction, float>
DqnCaffe::SelectAction(const InputFrames& last_frames, double epsilon, double v, double phi)
{
	double probability;
	std::pair<DqnAction, float> output;

	assert(epsilon >= 0.0 && epsilon <= 1.0);
	probability = (double) rand() / (double) RAND_MAX;

	std::vector<std::vector<float> > qs;
	output = SelectActionGreedily(last_frames, v, phi, &qs);
	std::cout << PrintQValues(qs.front(), legal_actions_);

	if (probability < epsilon) // Select randomly
	{
		int random_idx = rand() % legal_actions_.size();

		output.first = legal_actions_[random_idx];
		output.second = qs.front().at(random_idx); // value predicted by the network for this action

		std::cout << action_to_string(output.first) << " (random)" << std::endl;
		return output;
	}
	else
	{
		std::cout << action_to_string(output.first) << " (greedy)" << std::endl;
		return output;
	}
}


void
DqnCaffe::AddTransition(const Transition& transition)
{
	if (replay_memory_.size() == replay_memory_capacity_)
		replay_memory_.pop_front();

	replay_memory_.push_back(transition);
}


// only for debug
void
show_input_frames(InputFrames frames, int id)
{
	cv::Mat img = cv::Mat(cv::Size(DqnParams::kInputFrameCount * DqnParams::kCroppedFrameSize, DqnParams::kCroppedFrameSize), CV_8UC1);
	cv::Mat res = cv::Mat(cv::Size(2 * DqnParams::kInputFrameCount * DqnParams::kCroppedFrameSize, 2 * DqnParams::kCroppedFrameSize), CV_8UC1);


	for (int k = 0; k < DqnParams::kInputFrameCount; k++)
	{
		for (int i = 0; i < DqnParams::kCroppedFrameSize; i++)
		{
			for (int j = 0; j < DqnParams::kCroppedFrameSize; j++)
			{
				int frame_p = i * DqnParams::kCroppedFrameSize + j;
				int img_p = i * img.step + k * DqnParams::kCroppedFrameSize + j;

				if (isnan((double) frames[k]->at(frame_p)))
					exit(printf("Problem!!! frame %d row %d col %d\n", k, i, j));

				img.data[img_p] = frames[k]->at(frame_p);
			}
		}
	}

	static int first = 1;

	if (first == 1)
	{
		cv::namedWindow("frame 0");
		cv::namedWindow("frame 8");
		cv::namedWindow("frame 20");
		cv::namedWindow("frame 31");

		cv::moveWindow("frame 0", 800, 0);
		cv::moveWindow("frame 8", 800, 250);
		cv::moveWindow("frame 20", 800, 500);
		cv::moveWindow("frame 31", 800, 750);

		first = 0;
	}

	char name[64];
	sprintf(name, "frame %d", id);
	cv::resize(img, res, res.size());
	cv::imshow(/*"input frames"*/name, res);
	cv::waitKey(1);
	printf("Show %s\n", name);
}


void
DqnCaffe::Update()
{
	current_iter_++;

//	if (current_iter_ >= 20) exit(0);

	int i, j, random_transition_id;

	// DEBUG:
	// std::cout << "iteration: " << current_iter_ << std::endl;

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
	std::vector<float> phi_batch;
	std::vector<float> v_batch;

	/**
	 * OBS: The Q-learning update is given by: Qt(s, a) = Qt-1(s, a) + alpha * (imediate_reward + Qt-1(s', a) - Qt-1), where s'
	 *  is the next state, achieved by taking action a in state s. The loop below and the following call of SelectActionGreedily are
	 *  intended to compute Qt-1(s', a). This is why the inner loop starts with j=1, and the frame achieved AFTER the execution of the
	 *  action is pushed_back to the input frames, and these input frames are used to estimate the Q value of the best action in the next state.
	 */

	for (i = 0; i < (int) transitions.size(); i++)
	{
		int idx = transitions[i];
		Transition& transition = replay_memory_[idx];

		if (transition.is_final_state)
		{
			// This is a terminal state
			continue;
		}

		InputFrames target_last_frames;

		for (j = 1; j < DqnParams::kInputFrameCount; ++j)
			target_last_frames.push_back(transition.input_frames[j]);

		target_last_frames.push_back(transition.frame_after_action);

// DEBUG:
//		if (i == 0 || i == 8 || i == 20 || i == 31)
//			show_input_frames(target_last_frames, i);

		v_batch.push_back(transition.v);
		phi_batch.push_back(transition.phi);
		target_last_frames_batch.push_back(target_last_frames);
	}


//	static int nupdate = 0;
//	std::cout << "nupdate: " << nupdate << std::endl;
//	nupdate++;
//	cv::waitKey(1);

	std::vector<std::pair<DqnAction, float> > actions_and_values = SelectActionGreedily(target_last_frames_batch, v_batch, phi_batch);

// DEBUG:
//	printf("\na_and_v size:  %ld\n\n", actions_and_values.size());
//
//	for (int i = 0; i < actions_and_values.size(); i++)
//		std::cout << i << " Greedy: " << action_to_string(actions_and_values[i].first) << " " << actions_and_values[i].second << std::endl;
//
//	printf("\n");
//

// DEBUG:
//	int num_problems = 0;
//	for (int i = 0; i < actions_and_values.size(); i++)
//	{
//		if (actions_and_values[i].second > 10.0 && num_problems < 4)
//		{
//			printf("PROBLEM IN %d %lf %lf\n", i, v_batch[i], phi_batch[i]);
//			show_input_frames(target_last_frames_batch[i], i);
//			num_problems++;
//		}
//	}
//
//	if (num_problems)
//		cv::waitKey(-1);

	// frames_input: 4 frames
	static FramesLayerInputData frames_input;
	// target_input: expected rewards for each action -> this value is compared to the q-value predicted by the network and the difference is used to update the weights
	static TargetLayerInputData target_input;
	// filter_input: 0 for all actions except the performed for which the value is 1
	static FilterLayerInputData filter_input;
	static OdometryLayerInputData odometry_input;

	static int first = 1;

	if (first)
	{
		for (i = 0 ; i < frames_input.capacity(); i++)
			frames_input.push_back(0);

		for (i = 0 ; i < filter_input.capacity(); i++)
			filter_input.push_back(0);

		for (i = 0 ; i < target_input.capacity(); i++)
			target_input.push_back(0);

		for (i = 0 ; i < odometry_input.capacity(); i++)
			odometry_input.push_back(0);

		first = 0;
	}
	else
	{
		// Set all filters and target to 0 every iteration. In the
		// loop below, only the performed action in a transition and
		// the expected reward of this action will be turned on.
		for (i = 0 ; i < filter_input.size(); i++)
			filter_input[i] = 0;

		for (i = 0 ; i < target_input.size(); i++)
			target_input[i] = 0;

		for (i = 0 ; i < frames_input.size(); i++)
			frames_input[i] = 0;

		for (i = 0 ; i < odometry_input.size(); i++)
			odometry_input[i] = 0;
	}

	int transition_id_in_the_action_value_vector = 0;

	for (i = 0; i < DqnParams::kMinibatchSize; ++i)
	{
		if (replay_memory_.size() < transitions[i])
		{
			printf("ERRO GRAVISSIMO\n");
			exit(-1);
		}

		Transition& transition = replay_memory_[transitions[i]];
		DqnAction action = transition.action; //std::get < 1 > (transition);

		assert(static_cast<int>(action) < DqnParams::kOutputCount);

		double reward = transition.reward; // std::get < 2 > (transition);

		// ***********************************
		// WHY IS THIS IMPORTANT????
		// ***********************************
		//assert(reward >= -1.0 && reward <= 1.0);

		double target = 0.0;

		if (DqnParams::TrainingModel == DQN_Q_LEARNING)
		{
		// Q-value re-estimation (from Q-learning)
		if (!transition.is_final_state)
		{
				// @filipe: ATENCAO!! NAO APAGUE ESSE COMENTARIO!! A variavel "actions_and_values" eh menor que o numero de transicoes porque o Q-value do
				// proximo estado NAO eh calculado para estados finais. Por isso o indice "transition_id_in_the_action_value_vector" eh usado para acessa-la
				// ao inves do "i". NUNCA TROQUE O "transition_id_in_the_action_value_vector" ABAIXO PARA i!!! ISSO FARA O CODIGO ACESSAR POSICOES INVALIDAS
				// DO VETOR, NAO DARA NENHUM ERRO E A REDE USARA VALORES MALUCOS PARA SE CALCULAR SE ATUALIZAR!!!!
				target = reward + gamma_ * actions_and_values[transition_id_in_the_action_value_vector].second - transition.estimated_reward;
				transition_id_in_the_action_value_vector++;
			}
			else
				// @filipe: Isso nao eh problema? Imagine que o robo passou por uma pose "A" e eventualmente chegou ao goal. A acao realizada na pose A
				// deve se reforcada nesse caso porque ela levou ao goal. Agora, imagine que o carro fez um trajeto qualquer e parou em "A". Com
				// o codigo abaixo, vamos treinar a recompensa imediata em "A"... Eh como se estivessemos desvalorizando "A" dado que podemos a partir
				// deste estado podemos alcancar uma recompensa bem maior que a imediata. Nao seria uma boa ideia escolher o maximo entre a recompensa
				// imediata e a predicao da rede? TODO: Pensar se essa solucao nao causa problema nos casos finais normais (atingir o goal e bater) e
				// se ela for OK, implementar.
				target = reward;
		}
		else if (DqnParams::TrainingModel == DQN_DISCOUNTED_TOTAL_REWARD || DqnParams::TrainingModel == DQN_INFINITY_HORIZON_DISCOUNTED_MODEL)
		{
			target = transition.reward;
		}
		else
		{
			exit(printf("ERROR: Unknown training model!\n"));
		}

		assert(!isnan(target));

		target_input[i * DqnParams::kOutputCount + static_cast<int>(action)] = target;
		filter_input[i * DqnParams::kOutputCount + static_cast<int>(action)] = 1;

		odometry_input[2 * i + 0] = (float) transition.v;
		odometry_input[2 * i + 1] = (float) transition.phi;

		// DEBUG:
		///*std::cout*/ std::cout << "filter:" << action_to_string(action) << " reward: " << reward << " net estimation: " << actions_and_values[i].second << " target: " << target << std::endl;

		for (j = 0; j < DqnParams::kInputFrameCount; ++j)
		{
			FrameDataSp& frame_data = transition.input_frames[j]; //std::get < 0 > (transition)[j];

			// ******************************************************************************************************
			// @filipe: isso nao vai dar problema? O frame_data eh uchar e o frames_input eh float.......
			// ******************************************************************************************************
			std::copy(frame_data->begin(), frame_data->end(),
				frames_input.begin() + i * DqnParams::kInputDataSize + j * DqnParams::kCroppedFrameDataSize);
		}
	}

	InputDataIntoLayers(frames_input, target_input, filter_input, odometry_input);
	solver_->Step(1); // @filipe: checar se o parametro nao deveria ser 0. olhar o comentario na definicao da funcao.

	assert(!isnan(net_->layer_by_name("conv1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("conv2_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("ip1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("ip2_layer")->blobs().front()->data_at(1, 0, 0, 0)));

	// DEBUG: Log the first parameter of each hidden layer
//	std::cout << std::endl;
//	std::cout << "conv1:" << net_->layer_by_name("conv1_layer")->blobs().front()->data_at(1, 0, 0, 0) << std::endl;
//	std::cout << "conv2:" << net_->layer_by_name("conv2_layer")->blobs().front()->data_at(1, 0, 0, 0) << std::endl;
//	std::cout << "ip1:" << net_->layer_by_name("ip1_layer")->blobs().front()->data_at(1, 0, 0, 0) << std::endl;
//	std::cout	<< "ip2:" << net_->layer_by_name("ip2_layer")->blobs().front()->data_at(1, 0, 0, 0) << std::endl;
//	std::cout << std::endl;
//	//getchar();

//	//DEBUG:
//	std::cout << std::endl;
//	std::cout << "shape: " << net_->blob_by_name("target")->shape_string() << std::endl;
//

	static std::vector<float> max_diff;

	if (max_diff.size() == 0)
		for (int i = 0; i < legal_actions_.size(); i++, max_diff.push_back(0));

	for (i = 0; i < DqnParams::kMinibatchSize; i++)
	{
		for (j = 0; j < legal_actions_.size(); j++)
		{
			if (fabs(net_->blob_by_name("target")->data_at(i, j, 0, 0) - net_->blob_by_name("filtered_q_values")->data_at(i, j, 0, 0)) > max_diff[j])
				max_diff[j] = fabs(net_->blob_by_name("target")->data_at(i, j, 0, 0) - net_->blob_by_name("filtered_q_values")->data_at(i, j, 0, 0));

//			std::cout << "batch " << i << " command " << j << " net q values: " << net_->blob_by_name("filtered_q_values")->data_at(i, j, 0, 0) << " target: " <<
//					net_->blob_by_name("target")->data_at(i, j, 0, 0) << " reward: " << replay_memory_[transitions[i]].reward << " Qt-1 " << actions_and_values[i].second <<
//					" estimated_reward " << replay_memory_[transitions[i]].estimated_reward << std::endl;
		}

//		std::cout << std::endl;
	}

	std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t" << "Max Diff: " << std::endl;
	for (j = 0; j < legal_actions_.size(); j++)
		std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t" << action_to_string(legal_actions_[j]) << ": " << max_diff[j] << std::endl;

}


