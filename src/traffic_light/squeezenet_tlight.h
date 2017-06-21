
#ifndef CARMEN_SQUEEZENET_TLIGHT_H
#define CARMEN_SQUEEZENET_TLIGHT_H

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <vector>
#include <caffe/caffe.hpp>

class SqueezeNet
{
public:
	SqueezeNet(const std::string &model_file, const std::string& trained_file, int GPU, int device_id);
	// returns an image with the same dimensions of the input (including the channels).
	// The pixels of the output image are the pixel's classes.
	// Note: the classes are repeated in the channels.
	int Predict(const cv::Mat& img);
	// convert the mat of classes to a mat of colors (just for visualization)
	cv::Mat ClassesToColors(cv::Mat &img);

private:

	void WrapInputLayer(std::vector<cv::Mat>* input_channels);
	void Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels);

	boost::shared_ptr<caffe::Net<float>> net_;
	cv::Size input_geometry_;
	int num_channels_;

};

#endif
