
#ifndef CARMEN_SEMANTIC_SEGMENTATION_H
#define CARMEN_SEMANTIC_SEGMENTATION_H

#include <opencv/cv.h>
#include <opencv/highgui.h>

#define USE_OPENCV 1
#include <caffe/caffe.hpp>
#include <string>

class SegNet
{
	public:
		SegNet(const std::string& model_file, const std::string& trained_file, std::string classes_color_palette_file);
		// returns an image with the same dimensions of the input (including the channels).
		// The pixels of the output image are the pixel's classes.
		// Note: the classes are repeated in the channels.
		cv::Mat Predict(const cv::Mat& img);
		// convert the mat of classes to a mat of colors (just for visualization)
		cv::Mat ClassesToColors(cv::Mat &img);

	private:

		void WrapInputLayer(std::vector<cv::Mat>* input_channels);
		void Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels);

		boost::shared_ptr<caffe::Net<float>> net_;
		cv::Size input_geometry_;
		cv::Mat label_colours_;
		int num_channels_;

};

#endif
