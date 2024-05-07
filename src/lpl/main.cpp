#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "onnxruntime_inference.h"
#include <iostream>
#include <stdio.h>
void main() {
	// Opencv related
	cv::VideoCapture cap;
	cv::Mat Frame;
	//Model related
	std::vector<const char*> input_node_names = { "images" }; // Input node names
	std::vector<int64_t> input_dims = { 1, 3,640,640 };
	std::vector<const char*> output_node_names = { "output" }; // Output node names
	std::vector<Ort::Value> OutputTensor;					   // Holds the result of inference
	OnnxENV Env;
	YOLOv7 model;

	int numDetected = -1;

	//Set model settings
	model.SetSessionOptions(true);
	model.LoadWeights(&Env, L"yolov7-tiny.onnx");	// Load weight and create session
	model.SetInputDemensions(input_dims);
	model.SetInputNodeNames(&input_node_names);
	model.SetOutputNodeNames(&output_node_names);
	// open video capture
	bool error = cap.open("bottle_detection.mp4", cv::CAP_ANY);
	if (!cap.isOpened()) {
		std::cout << "ERROR! Unable to open file\n";
		return;
	}
	// Main loop
	error = cap.read(Frame);
	while (error) {
		if (Frame.empty() == true) break;
		numDetected = model.Inference(Frame, OutputTensor);
		float* Result = OutputTensor.front().GetTensorMutableData<float>();
		// Drawing the boxes
		for (int i = 0; i < numDetected; i += 7) {		// the output of this model is (number_of_detected , 7), thus the increment by 7
			float x = Result[i + 1] / 640 * (float)Frame.cols;
			float y = Result[i + 2] / 640 * (float)Frame.rows;
			float w = (Result[i + 3] - Result[i + 1]) / (float)640 * (float)Frame.cols;
			float h = (Result[i + 4] - Result[i + 2]) / (float)640 * (float)Frame.rows;
			cv::rectangle(Frame, cv::Rect2f(x, y, w, h), cv::Scalar(255, 0, 0), 2, 8, 0);
		}
		OutputTensor.clear();
		cv::waitKey(20);
		cv::imshow("result", Frame);	// Use a kalman filter if you think the boundary lines are too shakey

		error = cap.read(Frame);
	}

	cap.release();
}