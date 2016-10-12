
#include "DqnUtil.h"
#include "DqnNet.h"
#include "DqnParams.h"
#include <opencv/highgui.h>


template<typename Dtype>
bool DqnNet::_HasBlobSize(caffe::Blob<Dtype>& blob, int batch_size, int channels, int height, int width)
{
	return ((blob.num() == batch_size) &&
			(blob.channels() == channels) &&
			(blob.height() == height) &&
			(blob.width() == width));
}


void
DqnNet::_AddDatumToVector(vector<caffe::Datum> *datums, caffe::Datum d)
{
	datums->push_back(d);
}


caffe::Datum
DqnNet::_BuildEmptyDatum(int rows, int cols, int channels)
{
	caffe::Datum d;

	d.set_channels(channels);
	d.set_height(rows);
	d.set_width(cols);
	d.set_label(0);

	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
			for (int k = 0; k < channels; k++)
				d.add_float_data(0.0);

	return d;
}


caffe::Datum
DqnNet::_BuildInputDatum(vector<Mat*> &ms)
{
	int img_p, line_size;
	float pixel;
	caffe::Datum d;

	assert(ms.size() > 0);
	assert(ms[0]->rows == DQN_FRAME_DIM);
	assert(ms[0]->cols == DQN_FRAME_DIM);
	assert(ms[0]->channels() == DQN_FRAME_CHANNELS);

	d.set_channels(DQN_FRAME_CHANNELS * DQN_NUM_INPUT_FRAMES);
	d.set_height(DQN_FRAME_DIM);
	d.set_width(DQN_FRAME_DIM);
	d.set_label(0);

	line_size = DQN_FRAME_DIM * DQN_FRAME_CHANNELS;

	for (int i = 0; i < DQN_FRAME_DIM; i++)
	{
		for (int j = 0; j < DQN_FRAME_DIM; j++)
		{
			for (int n = 0; n < DQN_NUM_INPUT_FRAMES; n++)
			{
				for (int k = 0; k < DQN_FRAME_CHANNELS; k++)
				{
					if (n < ms.size())
					{
						img_p = i * line_size + j * DQN_FRAME_CHANNELS + k;
						pixel = (float) ms[n]->data[img_p];
						d.add_float_data(pixel);
					}
					else
						d.add_float_data(0);
				}
			}
		}
	}

	return d;
}


caffe::Datum
DqnNet::_BuildAddDataDatum(vector<float> *additional_data)
{
	int i, n;
	caffe::Datum d;

	d.set_channels(DQN_NUM_ADDITIONAL_DATA * DQN_NUM_COPIES_OF_ADDITIONAL_DATA);
	d.set_height(1);
	d.set_width(1);
	d.set_label(0);

	for (n = 0; n < DQN_NUM_COPIES_OF_ADDITIONAL_DATA; n++)
	{
		for (i = 0; i < additional_data->size(); i++)
			d.add_float_data(additional_data->at(i));
	}

	return d;
}


DqnNet::DqnNet(char *solver_file)
{
	_solver_file = solver_file;

	caffe::ReadProtoFromTextFileOrDie(solver_file, &solver_param_);
	solver_.reset(caffe::SolverRegistry<float>::CreateSolver(solver_param_));

	net_ = solver_->net();

	frames_input_layer_ = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(net_->layer_by_name(DQN_INPUT_LAYER_NAME));
	target_input_layer_ = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(net_->layer_by_name(DQN_TARGET_LAYER_NAME));
	filter_input_layer_ = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(net_->layer_by_name(DQN_FILTER_LAYER_NAME));
	additional_data_input_layer_ = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(net_->layer_by_name(DQN_ADD_DATA_LAYER_NAME));

	assert(frames_input_layer_);
	assert(target_input_layer_);
	assert(filter_input_layer_);
	assert(additional_data_input_layer_);

	assert(_HasBlobSize(*net_->blob_by_name(DQN_INPUT_BLOB_NAME), DQN_MINI_BATCH_SIZE, DQN_FRAME_CHANNELS * DQN_NUM_INPUT_FRAMES, DQN_FRAME_DIM, DQN_FRAME_DIM));
	assert(_HasBlobSize(*net_->blob_by_name(DQN_TARGET_BLOB_NAME), DQN_MINI_BATCH_SIZE, DQN_NUM_COMMANDS, 1, 1));
	assert(_HasBlobSize(*net_->blob_by_name(DQN_FILTER_BLOB_NAME), DQN_MINI_BATCH_SIZE, DQN_NUM_COMMANDS, 1, 1));
	assert(_HasBlobSize(*net_->blob_by_name(DQN_ADD_DATA_BLOB_NAME), DQN_MINI_BATCH_SIZE, DQN_TOTAL_ADDITIONAL_DATA_SIZE, 1,1));


	// add empty datums to fill the batch
	for (int i = 0; i < DQN_MINI_BATCH_SIZE; i++)
	{
		_AddDatumToVector(&dummy_target_datum_vector, _BuildEmptyDatum(1, 1, DQN_NUM_COMMANDS));
		_AddDatumToVector(&dummy_filter_datum_vector, _BuildEmptyDatum(1, 1, DQN_NUM_COMMANDS));
		_AddDatumToVector(&test_input_datum_vector, _BuildEmptyDatum(DQN_FRAME_DIM, DQN_FRAME_DIM, DQN_FRAME_CHANNELS * DQN_NUM_INPUT_FRAMES));
		_AddDatumToVector(&test_add_data_datum_vector, _BuildEmptyDatum(1, 1, DQN_TOTAL_ADDITIONAL_DATA_SIZE));

		_AddDatumToVector(&train_target_datum_vector, _BuildEmptyDatum(1, 1, DQN_NUM_COMMANDS));
		_AddDatumToVector(&train_filter_datum_vector, _BuildEmptyDatum(1, 1, DQN_NUM_COMMANDS));
		_AddDatumToVector(&train_input_datum_vector, _BuildEmptyDatum(DQN_FRAME_DIM, DQN_FRAME_DIM, DQN_FRAME_CHANNELS * DQN_NUM_INPUT_FRAMES));
		_AddDatumToVector(&train_add_data_datum_vector, _BuildEmptyDatum(1, 1, DQN_TOTAL_ADDITIONAL_DATA_SIZE));
	}

	//cv::namedWindow("1");
	//cv::namedWindow("2");
	//cv::namedWindow("3");
	//cv::namedWindow("4");
	//cv::namedWindow("5");
	//cv::namedWindow("6");
    //
	//cv::moveWindow("1", 100, 100);
	//cv::moveWindow("2", 300, 100);
	//cv::moveWindow("3", 500, 100);
	//cv::moveWindow("4", 100, 300);
	//cv::moveWindow("5", 300, 300);
	//cv::moveWindow("6", 500, 300);
}


std::pair<int, double>
DqnNet::SelectAction(vector<Mat*> input, vector<float> *additional_data, int show_output)
{
	int i;
	float loss;

	assert(input.size() == DQN_NUM_INPUT_FRAMES);
	assert(additional_data);
	assert(additional_data->size() == DQN_NUM_ADDITIONAL_DATA);

	if (show_output)
	{
		cv::imshow("1", *(input[0]));
		waitKey(1);
	}

	// check if it can be optimized
	// OBS: As instrucoes abaixo assumem que os vetores de datums estao preenchidos com zero e sempre
	// sobrescrevem apenas a primeira posicao do vetor.
	test_input_datum_vector[0].CopyFrom(_BuildInputDatum(input));
	test_add_data_datum_vector[0].CopyFrom(_BuildAddDataDatum(additional_data));

	frames_input_layer_->AddDatumVector(test_input_datum_vector);
	additional_data_input_layer_->AddDatumVector(test_add_data_datum_vector);
	target_input_layer_->AddDatumVector(dummy_target_datum_vector);
	filter_input_layer_->AddDatumVector(dummy_filter_datum_vector);

	net_->ForwardPrefilled(&loss);

	std::pair<int, double> action_and_reward(-1, -DBL_MAX);

	// select the action with max. Q
	const float *output = net_->blob_by_name(DQN_OUTPUT_BLOB_NAME)->cpu_data();

	for (i = 0; i < DQN_NUM_COMMANDS; i++)
	{
		if (show_output)
			printf("%d => %f\n", i, output[i]);

		if (output[i] > action_and_reward.second)
		{
			action_and_reward.first = i;
			action_and_reward.second = output[i];
		}
	}

	if (show_output)
		printf("SEL: %d => %f\n", action_and_reward.first, action_and_reward.second);

	return action_and_reward;
}


//void
//DqnNet::_TrainRewardWithDecay(DqnEpisode *episode)
//{
//	int i, j, k, base_pos, sample_pos;
//	float loss;
//
//	int nt = episode->GetInteractions()->size() - DQN_NUM_INPUT_FRAMES;
//	int num_mini_batches = nt / DQN_MINI_BATCH_SIZE;
//
//	// to make sure that the final interactions will be trained
//	if (nt % DQN_MINI_BATCH_SIZE != 0)
//		num_mini_batches++;
//
//	for (i = 0; i < num_mini_batches; i += 1)
//	{
//		if (i % 20 == 0) printf("batch %d de %d BS: %d EPI SISE: %ld\n", i, num_mini_batches, DQN_MINI_BATCH_SIZE, episode->GetInteractions()->size());
//
//		// jump the first interactions because we need a minimum number of training samples
//		int base_pos = i * DQN_MINI_BATCH_SIZE + DQN_NUM_INPUT_FRAMES;
//
//		// Fill the train datums
//		for (j = 0; j < DQN_MINI_BATCH_SIZE; j++)
//		{
//			sample_pos = base_pos + j;
//
//			// the last mini batch will be incomplete, so we fill it with random interactions
//			if (sample_pos >= episode->GetInteractions()->size())
//				sample_pos = (rand() % (episode->GetInteractions()->size() - DQN_NUM_INPUT_FRAMES)) + DQN_NUM_INPUT_FRAMES;
//
//			vector<Mat*> inputs;
//
//			inputs.push_back(episode->GetInteractions()->at(0)->input);
//			for (k = (DQN_NUM_INPUT_FRAMES - 2); k >= 0; k--)
//				inputs.push_back(episode->GetInteractions()->at(sample_pos - k)->input);
//
//			cv::imshow("1", *(inputs[0]));
//			cv::imshow("2", *(inputs[1]));
//			cv::imshow("3", *(inputs[2]));
//			cv::imshow("4", *(inputs[3]));
//			cv::imshow("5", *(inputs[4]));
//			cv::imshow("6", *(inputs[5]));
//
//			static int step = 0;
//			char c = ' ';
//
//			if (step) c = cv::waitKey(-1);
//			else c = cv::waitKey(3);
//
//			if (c == 's') step = !step;
//
//			show_control(episode->GetInteractions()->at(sample_pos)->buttons_states,
//					episode->GetInteractions()->at(sample_pos)->action);
//
//			// input datum
//			train_input_datum_vector[j].CopyFrom(_BuildInputDatum(inputs));
//
//			// additional data datum
//			train_add_data_datum_vector[j].CopyFrom(_BuildAddDataDatum(
//					episode->GetInteractions()->at(sample_pos)->buttons_states,
//					episode->GetInteractions()->at(sample_pos)->last_commands,
//					episode->GetInteractions()->at(sample_pos)->rom_info));
//
//			//printf("buttons: ");
//			//for (k = 0; k < TOTAL_BUTTONS; k++)
//			//	printf("%d ", episode->GetInteractions()->at(sample_pos)->buttons_states[k]);
//			//printf("\n");
//			//printf("commands: \n");
//			//for (k = 0; k < episode->GetInteractions()->at(sample_pos)->last_commands->size(); k++)
//			//	printf("%s ", action_to_string(Action(episode->GetInteractions()->at(sample_pos)->last_commands->at(k))).c_str());
//			//printf("\n");
//			//printf("Info: ");
//			//for (k = 0; k < episode->GetInteractions()->at(sample_pos)->rom_info->size(); k++)
//			//	printf("%d ", episode->GetInteractions()->at(sample_pos)->rom_info->at(k));
//			//printf("\n");
//			//waitKey(-1);
//
//			// target and filter datums (set all outputs to zero and then set the action that we want to train)
//			for (k = 0; k < DQN_NUM_COMMANDS; k++)
//			{
//				train_target_datum_vector[j].set_float_data(k, 0);
//				train_filter_datum_vector[j].set_float_data(k, 0);
//			}
//
//			// the target is the decayed reward computed previously
//			double target = episode->GetInteractions()->at(sample_pos)->immediate_reward;
//
//			// set the action performed in the episode and its respective value
//			train_target_datum_vector[j].set_float_data(episode->GetInteractions()->at(sample_pos)->action, target);
//			train_filter_datum_vector[j].set_float_data(episode->GetInteractions()->at(sample_pos)->action, 1.0);
//
//			if (i % 20 == 0 && j == 0)
//			{
//				printf("%d de %ld: REW: %.4lf TARGET: %.4lf\n", sample_pos,
//						episode->GetInteractions()->size() - 1,
//						episode->GetInteractions()->at(sample_pos)->immediate_reward,
//						target);
//			}
//		}
//
//		// add the datums to the net
//		frames_input_layer_->AddDatumVector(train_input_datum_vector);
//		additional_data_input_layer_->AddDatumVector(train_add_data_datum_vector);
//		target_input_layer_->AddDatumVector(train_target_datum_vector);
//		filter_input_layer_->AddDatumVector(train_filter_datum_vector);
//
//		// train
//		solver_->Step(1);
//
//		printf("\nQs Pred: ");
//		for (k = 0; k < DQN_NUM_COMMANDS; k++)
//			printf("%.3lf ", net_->blob_by_name("q_values")->cpu_data()[k]);
//		printf("\nQs Pred Filtered: ");
//		for (k = 0; k < DQN_NUM_COMMANDS; k++)
//			printf("%.3lf ", net_->blob_by_name("filtered_q_values")->cpu_data()[k]);
//		printf("\nTarget: ");
//		for (k = 0; k < DQN_NUM_COMMANDS; k++)
//			printf("%.3lf %s ", net_->blob_by_name("target")->cpu_data()[k], (net_->blob_by_name("target")->cpu_data()[k] != 0) ? ("<<<<<<<<<<<<<<<<") : (""));
//		printf("\n");
//
//		assert(!isnan(net_->layer_by_name("conv1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//		assert(!isnan(net_->layer_by_name("conv2_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//		assert(!isnan(net_->layer_by_name("ip1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//		assert(!isnan(net_->layer_by_name("ip1.5_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//		assert(!isnan(net_->layer_by_name("ip2_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//	}
//}


//void
//DqnNet::_TrainQLearning(DqnEpisode *episode)
//{
//	int i, j, k, base_pos, sample_pos;
//	float loss;
//
//	int nt = episode->GetInteractions()->size() - DQN_NUM_INPUT_FRAMES;
//	int num_mini_batches = nt / DQN_MINI_BATCH_SIZE;
//
//	// to make sure that the final interactions will be trained
//	if (nt % DQN_MINI_BATCH_SIZE != 0)
//		num_mini_batches++;
//
//	for (i = 0; i < num_mini_batches; i += 1)
//	{
//		if (i % 20 == 0) printf("batch %d de %d BS: %d EPI SISE: %ld\n", i, num_mini_batches, DQN_MINI_BATCH_SIZE, episode->GetInteractions()->size());
//
//		// jump the first interactions because we need a minimum number of training samples
//		int base_pos = i * DQN_MINI_BATCH_SIZE + DQN_NUM_INPUT_FRAMES;
//
//		// Fill the train datums
//		for (j = 0; j < DQN_MINI_BATCH_SIZE; j++)
//		{
//			sample_pos = base_pos + j;
//
//			// the last mini batch will be incomplete, so we fill it with random interactions
//			if (sample_pos >= episode->GetInteractions()->size())
//				sample_pos = (rand() % (episode->GetInteractions()->size() - DQN_NUM_INPUT_FRAMES)) + DQN_NUM_INPUT_FRAMES;
//
//			vector<Mat*> inputs;
//
//			inputs.push_back(episode->GetInteractions()->at(0)->input);
//
//			for (k = (DQN_NUM_INPUT_FRAMES - 2); k >= 0; k--)
//				inputs.push_back(episode->GetInteractions()->at(sample_pos - k)->input);
//
//			cv::imshow("1", *(inputs[0]));
//			cv::imshow("2", *(inputs[1]));
//			cv::imshow("3", *(inputs[2]));
//			cv::imshow("4", *(inputs[3]));
//			cv::imshow("5", *(inputs[4]));
//			cv::imshow("6", *(inputs[5]));
//			cv::waitKey(1);
//
//			show_control(episode->GetInteractions()->at(sample_pos)->buttons_states,
//					episode->GetInteractions()->at(sample_pos)->action);
//
//			train_input_datum_vector[j].CopyFrom(_BuildInputDatum(inputs));
//
//			train_add_data_datum_vector[j].CopyFrom(_BuildAddDataDatum(
//					episode->GetInteractions()->at(sample_pos)->buttons_states,
//					episode->GetInteractions()->at(sample_pos)->last_commands,
//					episode->GetInteractions()->at(sample_pos)->rom_info));
//
//			train_target_datum_vector[j].CopyFrom(dummy_target_datum_vector[0]);
//			train_filter_datum_vector[j].CopyFrom(dummy_filter_datum_vector[0]);
//
//			/* Compute the future reward using the model*/
//			double target;
//
//			// if it is the last sample, we train only the immediate reward
//			if (sample_pos == episode->GetInteractions()->size() - 1)
//			{
//				target = episode->GetInteractions()->at(sample_pos)->immediate_reward;
//
//				if (i % 20 == 0 && j == 0)
//					printf("%d de %ld: REW: %.4lf FUT: ------ TARGET: %.4lf\n", sample_pos,
//							episode->GetInteractions()->size() - 1,
//							episode->GetInteractions()->at(sample_pos)->immediate_reward,
//							target);
//			}
//			// else we estimate the future reward and compute the temporal difference
//			else
//			{
//				vector<Mat*> future_inputs;
//
//				future_inputs.push_back(episode->GetInteractions()->at(0)->input);
//
//				for (k = (DQN_NUM_INPUT_FRAMES - 3); k >= 0; k--)
//					future_inputs.push_back(episode->GetInteractions()->at(sample_pos - k)->input);
//
//				future_inputs.push_back(episode->GetInteractions()->at(sample_pos + 1)->input);
//
//				// OBS: I am performing a forward for each sample of the batch and it is obviously a inefficient solution.
//				// I kept it so far to maintain the code clean. In the future I will change it.
//				std::pair<int, double> action_and_reward = SelectAction(future_inputs,
//						episode->GetInteractions()->at(sample_pos + 1)->buttons_states,
//						episode->GetInteractions()->at(sample_pos + 1)->last_commands,
//						episode->GetInteractions()->at(sample_pos + 1)->rom_info);
//
//				target = episode->GetInteractions()->at(sample_pos)->immediate_reward + DQN_GAMMA * action_and_reward.second;
//
//				if (i % 20 == 0 && j == 0)
//				{
//					printf("%d de %ld: REW: %.4lf FUT: %.4lf TARGET: %.4lf\n", sample_pos, episode->GetInteractions()->size() - 1,
//							episode->GetInteractions()->at(sample_pos)->immediate_reward,
//							action_and_reward.second,
//							target);
//				}
//			}
//
//			train_target_datum_vector[j].set_float_data(episode->GetInteractions()->at(sample_pos)->action, target);
//			train_filter_datum_vector[j].set_float_data(episode->GetInteractions()->at(sample_pos)->action, 1.0);
//		}
//
//		frames_input_layer_->AddDatumVector(train_input_datum_vector);
//		additional_data_input_layer_->AddDatumVector(train_add_data_datum_vector);
//		target_input_layer_->AddDatumVector(train_target_datum_vector);
//		filter_input_layer_->AddDatumVector(train_filter_datum_vector);
//
//		solver_->Step(4);
//
//		assert(!isnan(net_->layer_by_name("conv1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//		assert(!isnan(net_->layer_by_name("conv2_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//		assert(!isnan(net_->layer_by_name("ip1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//		assert(!isnan(net_->layer_by_name("ip1.5_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//		assert(!isnan(net_->layer_by_name("ip2_layer")->blobs().front()->data_at(1, 0, 0, 0)));
//	}
//}


void
DqnNet::_TrainTransitionRewardWithDecay(DqnEpisode *episode, int transition_id)
{
	// ********************************************************************1
	// OBS: ASSUME QUE O BATCH EH DE TAMANHO 1. VOU CORRIGIR EM BREVE!
	// ********************************************************************
	int j, k, /*base_pos,*/ sample_pos;
	//float loss;

	j = 0;

	sample_pos = transition_id;

	// the last mini batch will be incomplete, so we fill it with random interactions
	if (sample_pos >= episode->GetInteractions()->size())
		sample_pos = (rand() % (episode->GetInteractions()->size() - DQN_NUM_INPUT_FRAMES)) + DQN_NUM_INPUT_FRAMES;

	vector<Mat*> inputs;

	//inputs.push_back(episode->GetInteractions()->at(0)->input);
	for (k = (DQN_NUM_INPUT_FRAMES - 1); k >= 0; k--)
		inputs.push_back(episode->GetInteractions()->at(sample_pos - k)->input);

	cv::imshow("1", *(inputs[0]));
	//cv::imshow("2", *(inputs[1]));
	//cv::imshow("3", *(inputs[2]));
	//cv::imshow("4", *(inputs[3]));
	//cv::imshow("5", *(inputs[4]));
	//cv::imshow("6", *(inputs[5]));

	static int step = 0;
	char c = ' ';

	if (step) c = cv::waitKey(-1);
	else c = cv::waitKey(5);

	if (c == 's') step = !step;

	//show_control(episode->GetInteractions()->at(sample_pos)->buttons_states,
			//episode->GetInteractions()->at(sample_pos)->action);

	// input datum
	train_input_datum_vector[j].CopyFrom(_BuildInputDatum(inputs));

	// additional data datum
	train_add_data_datum_vector[j].CopyFrom(_BuildAddDataDatum(
			episode->GetInteractions()->at(sample_pos)->AdditionalDataVector()
			/*episode->GetInteractions()->at(sample_pos)->buttons_states,
			episode->GetInteractions()->at(sample_pos)->last_commands,
			episode->GetInteractions()->at(sample_pos)->rom_info)*/ ));

	//printf("Additional Data: ");
    //
	//for (k = 0; k < DQN_NUM_ADDITIONAL_DATA; k++)
	//{
	//	if (k == 5 || k == 15) printf("\n");
	//	printf("%d %f ", k, episode->GetInteractions()->at(sample_pos)->AdditionalDataVector()->at(k));
	//}
    //
	//printf("\n");

	//printf("commands: \n");
	//for (k = 0; k < episode->GetInteractions()->at(sample_pos)->last_commands->size(); k++)
	//	printf("%s ", action_to_string(Action(episode->GetInteractions()->at(sample_pos)->last_commands->at(k))).c_str());
	//printf("\n");
	//printf("Info: ");
	//for (k = 0; k < episode->GetInteractions()->at(sample_pos)->rom_info->size(); k++)
	//	printf("%d ", episode->GetInteractions()->at(sample_pos)->rom_info->at(k));
	//printf("\n");
	//waitKey(-1);

	// target and filter datums (set all outputs to zero and then set the action that we want to train)
	for (k = 0; k < DQN_NUM_COMMANDS; k++)
	{
		train_target_datum_vector[j].set_float_data(k, 0);
		train_filter_datum_vector[j].set_float_data(k, 0);
	}

	// the target is the decayed reward computed previously
	double target = episode->GetInteractions()->at(sample_pos)->immediate_reward;

	// set the action performed in the episode and its respective value
	train_target_datum_vector[j].set_float_data(episode->GetInteractions()->at(sample_pos)->action, target);
	train_filter_datum_vector[j].set_float_data(episode->GetInteractions()->at(sample_pos)->action, 1.0);

	/*if (i % 20 == 0 && j == 0)
	{
		printf("%d de %ld: REW: %.4lf TARGET: %.4lf\n", sample_pos,
				episode->GetInteractions()->size() - 1,
				episode->GetInteractions()->at(sample_pos)->immediate_reward,
				target);
	}*/

	// add the datums to the net
	frames_input_layer_->AddDatumVector(train_input_datum_vector);
	additional_data_input_layer_->AddDatumVector(train_add_data_datum_vector);
	target_input_layer_->AddDatumVector(train_target_datum_vector);
	filter_input_layer_->AddDatumVector(train_filter_datum_vector);

	// train
	solver_->Step(4);

	//printf("\nQs Pred: ");
	//for (k = 0; k < DQN_NUM_COMMANDS; k++)
	//	printf("%.3lf ", net_->blob_by_name("q_values")->cpu_data()[k]);
	//printf("\nQs Pred Filtered: ");
	//for (k = 0; k < DQN_NUM_COMMANDS; k++)
	//	printf("%.3lf ", net_->blob_by_name("filtered_q_values")->cpu_data()[k]);
	//printf("\nTarget: ");
	//for (k = 0; k < DQN_NUM_COMMANDS; k++)
	//	printf("%.3lf %s ", net_->blob_by_name("target")->cpu_data()[k], (net_->blob_by_name("target")->cpu_data()[k] != 0) ? ("<<<<<<<<<<<<<<<<") : (""));
	//printf("\n");
	//getchar();

	assert(!isnan(net_->layer_by_name("conv1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("conv2_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("ip1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("ip1.5_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("ip2_layer")->blobs().front()->data_at(1, 0, 0, 0)));
}


void
DqnNet::_TrainTransitionQLearning(DqnEpisode *episode, int transition_id)
{
	// ********************************************************************1
	// OBS: ASSUME QUE O BATCH EH DE TAMANHO 1. VOU CORRIGIR EM BREVE!
	// ********************************************************************
	int j, k, /*base_pos,*/ sample_pos;
	//float loss;

	sample_pos = transition_id;
	j = 0;

	// the last mini batch will be incomplete, so we fill it with random interactions
	if (sample_pos >= episode->GetInteractions()->size())
		sample_pos = (rand() % (episode->GetInteractions()->size() - DQN_NUM_INPUT_FRAMES)) + DQN_NUM_INPUT_FRAMES;

	vector<Mat*> inputs;

	for (k = (DQN_NUM_INPUT_FRAMES - 1); k >= 0; k--)
		inputs.push_back(episode->GetInteractions()->at(sample_pos - k)->input);

	cv::imshow("1", *(inputs[0]));
	//cv::imshow("2", *(inputs[1]));
	//cv::imshow("3", *(inputs[2]));
	//cv::imshow("4", *(inputs[3]));
	//cv::imshow("5", *(inputs[4]));
	//cv::imshow("6", *(inputs[5]));
	static int step = 0;
	char c = ' ';

	if (step) c = cv::waitKey(-1);
	else c = cv::waitKey(5);

	if (c == 's') step = !step;

	//show_control(episode->GetInteractions()->at(sample_pos)->buttons_states,
			//episode->GetInteractions()->at(sample_pos)->action);

	train_input_datum_vector[j].CopyFrom(_BuildInputDatum(inputs));

	train_add_data_datum_vector[j].CopyFrom(_BuildAddDataDatum(
			episode->GetInteractions()->at(sample_pos)->AdditionalDataVector()
			/*episode->GetInteractions()->at(sample_pos)->buttons_states,
			episode->GetInteractions()->at(sample_pos)->last_commands,
			episode->GetInteractions()->at(sample_pos)->rom_info)*/ ));

	train_target_datum_vector[j].CopyFrom(dummy_target_datum_vector[0]);
	train_filter_datum_vector[j].CopyFrom(dummy_filter_datum_vector[0]);

	/* Compute the future reward using the model*/
	double target;

	// if it is the last sample, we train only the immediate reward
	if (sample_pos == episode->GetInteractions()->size() - 1)
	{
		target = episode->GetInteractions()->at(sample_pos)->immediate_reward;

		/*
		if (i % 20 == 0 && j == 0)
			printf("%d de %ld: REW: %.4lf FUT: ------ TARGET: %.4lf\n", sample_pos,
					episode->GetInteractions()->size() - 1,
					episode->GetInteractions()->at(sample_pos)->immediate_reward,
					target);
		*/
	}
	// else we estimate the future reward and compute the temporal difference
	else
	{
		vector<Mat*> future_inputs;

		//future_inputs.push_back(episode->GetInteractions()->at(0)->input);

		for (k = (DQN_NUM_INPUT_FRAMES - 2); k >= 0; k--)
			future_inputs.push_back(episode->GetInteractions()->at(sample_pos - k)->input);
		future_inputs.push_back(episode->GetInteractions()->at(sample_pos + 1)->input);

		//imshow("future", *episode->GetInteractions()->at(sample_pos + 1)->input);
		//waitKey(1);

		// OBS: I am performing a forward for each sample of the batch and it is obviously a inefficient solution.
		// I kept it so far to maintain the code clean. In the future I will change it.
		std::pair<int, double> action_and_reward = SelectAction(future_inputs,
				episode->GetInteractions()->at(sample_pos + 1)->AdditionalDataVector(), 0
				/*buttons_states,
				episode->GetInteractions()->at(sample_pos + 1)->last_commands,
				episode->GetInteractions()->at(sample_pos + 1)->rom_info*/ );

		target = episode->GetInteractions()->at(sample_pos)->immediate_reward + DQN_GAMMA * action_and_reward.second;

		/*if (i % 20 == 0 && j == 0)
		{
			printf("%d de %ld: REW: %.4lf FUT: %.4lf TARGET: %.4lf\n", sample_pos, episode->GetInteractions()->size() - 1,
					episode->GetInteractions()->at(sample_pos)->immediate_reward,
					action_and_reward.second,
					target);
		}*/
	}

	train_target_datum_vector[j].set_float_data(episode->GetInteractions()->at(sample_pos)->action, target);
	train_filter_datum_vector[j].set_float_data(episode->GetInteractions()->at(sample_pos)->action, 1.0);

	frames_input_layer_->AddDatumVector(train_input_datum_vector);
	additional_data_input_layer_->AddDatumVector(train_add_data_datum_vector);
	target_input_layer_->AddDatumVector(train_target_datum_vector);
	filter_input_layer_->AddDatumVector(train_filter_datum_vector);

	solver_->Step(4);

	assert(!isnan(net_->layer_by_name("conv1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("conv2_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("ip1_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("ip1.5_layer")->blobs().front()->data_at(1, 0, 0, 0)));
	assert(!isnan(net_->layer_by_name("ip2_layer")->blobs().front()->data_at(1, 0, 0, 0)));
}


//void
//DqnNet::Train(DqnEpisode *episode)
//{
//	if (DQN_TRAINING_MODE == DQN_MODE_Q_LEARNING)
//		_TrainQLearning(episode);
//	else if (DQN_TRAINING_MODE == DQN_MODE_REWARD_WITH_DECAY)
//		_TrainRewardWithDecay(episode);
//	else
//		exit(printf("Training mode not found..."));
//}


void
DqnNet::TrainTransition(DqnEpisode *episode, int transition_id)
{
	if (DQN_TRAINING_MODE == DQN_MODE_Q_LEARNING)
		_TrainTransitionQLearning(episode, transition_id);
	else if (DQN_TRAINING_MODE == DQN_MODE_REWARD_WITH_DECAY)
		_TrainTransitionRewardWithDecay(episode, transition_id);
	else
		exit(printf("Training mode not found..."));
}


void
DqnNet::SaveTrain()
{
	solver_->Snapshot();
}


void
DqnNet::LoadTrain(char *caffemodel_file)
{
	net_->CopyTrainedLayersFrom(caffemodel_file);
}

