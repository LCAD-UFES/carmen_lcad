#include "squeezenet_tlight.h"

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>

using namespace caffe;

SqueezeNet::SqueezeNet(const std::string& model_file, const std::string& trained_file, int GPU, int device_id)
{
	//Setting CPU or GPU
	if (GPU)
	{
		Caffe::set_mode(Caffe::GPU);
		Caffe::SetDevice(device_id);
		printf("GPU mode device: %d\n", device_id);
	}
	else
	{
		Caffe::set_mode(Caffe::CPU);
		printf("CPU mode\n");
	}

	/* Load the network. */
	net_.reset(new Net<float>(model_file, TEST)); //deploy.prototxt
	net_->CopyTrainedLayersFrom(trained_file); // cafemodel

	CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
	CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

	Blob<float>* input_layer = net_->input_blobs()[0];
	num_channels_ = input_layer->channels();
	CHECK(num_channels_ == 3) << "Input layer should have 3 channels.";
	input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
}

/**
 *  none = 0,
 * 	red = 1,
 * 	green = 2
 */
int
SqueezeNet::Predict(const cv::Mat& img)
{
	int class_predicted = 0;
	std::vector<float> result_prediction;
	//Blob<float>* input_layer = net_->input_blobs()[0];
//	input_layer->Reshape(1, num_channels_, input_geometry_.height, input_geometry_.width);

	/* Forward dimension change to all layers. */
//	net_->Reshape();

	std::vector<cv::Mat> input_channels;
	WrapInputLayer(&input_channels);

	Preprocess(img, &input_channels);
	net_->Forward(0);

	/* Copy the output layer to a std::vector */
	  Blob<float>* output_layer = net_->output_blobs()[0];
	  const float* begin = output_layer->cpu_data();
	  const float* end = begin + output_layer->channels();
	  std::vector<float> class_prob(begin, end);

	  //argmax to get the class number with max probability
	  class_predicted = std::distance(class_prob.begin(),
			  std::max_element(class_prob.begin(), class_prob.end()));

	  return class_predicted;
}


/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void
SqueezeNet::WrapInputLayer(std::vector<cv::Mat>* input_channels)
{
	Blob<float>* input_layer = net_->input_blobs()[0];

	int width = input_layer->width();
	int height = input_layer->height();

	float* input_data = input_layer->mutable_cpu_data();

	for (int i = 0; i < input_layer->channels(); ++i)
	{
		cv::Mat channel(height, width, CV_32FC1, input_data);
		input_channels->push_back(channel);
		input_data += width * height;
	}
}


void
SqueezeNet::Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels)
{
	/* Convert the input image to the input image format of the network. */
	cv::Mat sample;

	if (img.channels() == 3 && num_channels_ == 1)
		cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
	else if (img.channels() == 4 && num_channels_ == 1)
		cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
	else if (img.channels() == 4 && num_channels_ == 3)
		cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
	else if (img.channels() == 1 && num_channels_ == 3)
		cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
	else
		sample = img;

	cv::Mat sample_resized;
	if (sample.size() != input_geometry_)
		cv::resize(sample, sample_resized, input_geometry_);
	else
		sample_resized = sample;

	cv::Mat sample_float;

	if (num_channels_ == 3)
		sample_resized.convertTo(sample_float, CV_32FC3);
	else
		sample_resized.convertTo(sample_float, CV_32FC1);

	/* This operation will write the separate BGR planes directly to the
	 * input layer of the network because it is wrapped by the cv::Mat
	 * objects in input_channels. */
	cv::split(sample_float, *input_channels);

	CHECK(reinterpret_cast<float*>(input_channels->at(0).data) == net_->input_blobs()[0]->cpu_data())
		<< "Input channels are not wrapping the input layer of the network.";
}
