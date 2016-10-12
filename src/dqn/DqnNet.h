
#ifndef DQNNET_H_
#define DQNNET_H_

#include "DqnEpisode.h"

#include <caffe/caffe.hpp>
#include <caffe/solver.hpp>
#include <caffe/sgd_solvers.hpp>
#include <caffe/util/io.hpp>
#include <caffe/layers/memory_data_layer.hpp>

class DqnNet
{
	char *_solver_file;

	boost::shared_ptr<caffe::Solver<float>> solver_;
	boost::shared_ptr<caffe::Net<float>> net_;
	caffe::SolverParameter solver_param_;

	boost::shared_ptr<caffe::Blob<float>> q_values_blob_;

	boost::shared_ptr<caffe::MemoryDataLayer<float>> frames_input_layer_;
	boost::shared_ptr<caffe::MemoryDataLayer<float>> target_input_layer_;
	boost::shared_ptr<caffe::MemoryDataLayer<float>> filter_input_layer_;
	boost::shared_ptr<caffe::MemoryDataLayer<float>> additional_data_input_layer_;

	vector<caffe::Datum> dummy_target_datum_vector;
	vector<caffe::Datum> dummy_filter_datum_vector;
	vector<caffe::Datum> test_input_datum_vector;
	vector<caffe::Datum> test_add_data_datum_vector;

	vector<caffe::Datum> train_target_datum_vector;
	vector<caffe::Datum> train_filter_datum_vector;
	vector<caffe::Datum> train_input_datum_vector;
	vector<caffe::Datum> train_add_data_datum_vector;

	template<typename Dtype>
	bool _HasBlobSize(caffe::Blob<Dtype>& blob, int batch, int channels, int height, int width);

	void _AddDatumToVector(vector<caffe::Datum> *datums, caffe::Datum d);

	caffe::Datum _BuildEmptyDatum(int rows, int cols, int channels);
	caffe::Datum _BuildInputDatum(vector<Mat*> &m);
	caffe::Datum _BuildAddDataDatum(vector<float> *additional_data);

	//void _TrainQLearning(DqnEpisode *episode);
	//void _TrainRewardWithDecay(DqnEpisode *episode);

	void _TrainTransitionQLearning(DqnEpisode *episode, int transition_id);
	void _TrainTransitionRewardWithDecay(DqnEpisode *episode, int transition_id);

	public:

		DqnNet(char *solver_file);

		// the first element is the action, and the second the estimated reward
		std::pair<int, double> SelectAction(vector<Mat*> input,
				vector<float> *additional_data, int show_output = 1);

		//void Train(DqnEpisode *episode);
		void TrainTransition(DqnEpisode *episode, int transition_id);
		
		void SaveTrain();
		void LoadTrain(char *filename);
};

#endif
