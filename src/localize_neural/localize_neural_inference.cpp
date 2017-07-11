#include "localize_neural_inference.h"

#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/graph/default_device.h>

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

static tensorflow::Session* session = NULL;


void
load_network(const char *saved_network)
{
	const std::string device = "/gpu:0";
	tensorflow::GraphDef graph_def;
	tensorflow::SessionOptions opts;
	tensorflow::Status status;

	status = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), saved_network, &graph_def);
	if (!status.ok())
	{
		std::cout << status.ToString() << "\n";
	}
	else
	{
		std::cout << "Network successfully read.\n";

		tensorflow::graph::SetDefaultDevice(device, &graph_def);

		opts.config.mutable_gpu_options()->set_per_process_gpu_memory_fraction(0.7);
		opts.config.mutable_gpu_options()->set_allow_growth(true);

		status = tensorflow::NewSession(opts, &session);
		if (!status.ok())
		{
			std::cout << status.ToString() << "\n";
		}
		else
		{
			std::cout << "Session successfully created.\n";

			status = session->Create(graph_def);
			if (!status.ok())
			{
				std::cout << status.ToString() << "\n";
			}
			else
			{
				std::cout << "Network successfully loaded.\n";
			}
		}
	}
}


void
initialize_tensorflow(const char * saved_network)
{
	load_network(saved_network);
}


void
finalize_tensorflow()
{
	if (session != NULL)
	{
		session->Close();
		delete session;
		session = NULL;
	}
}


void
copy_tensor(tensorflow::Tensor &input_tensor, const char *keyframe_data, const char *curframe_data, int height, int width, int depth)
{
	auto input_tensor_mapped = input_tensor.tensor<float, 4>();

	// copying the data into the corresponding tensor
	for (int y = 0; y < height; y++)
	{
		const char* keyframe_row = keyframe_data + (y * width * depth);
		const char* curframe_row = curframe_data + (y * width * depth);
		for (int x = 0; x < width; x++)
		{
			const char* keyframe_pixel = keyframe_row + (x * depth);
			const char* curframe_pixel = curframe_row + (x * depth);
			for (int c = 0; c < depth; c++)
			{
				const char* keyframe_value = keyframe_pixel + c;
				const char* curframe_value = curframe_pixel + c;
				input_tensor_mapped(0, y, x, c) = (float)(*keyframe_value);
				input_tensor_mapped(0, y, x, c+3) = (float)(*curframe_value);
			}
		}
	}
}


void
copy_image (const char *rgb_buffer, IplImage *img, int width, int height)
{
	for(int i = 0; i < (height * width); i++)
	{
		/**
		 * A imagem da bumblebee usa o formato rgb-rgb-rgb, enquanto
		 * a imagem da opencv usa o formato bgr-bgr-bgr. As linhas
		 * abaixo fazem essa conversao.
		 */
		img->imageData[3 * i + 0] = rgb_buffer[3 * i + 2];
		img->imageData[3 * i + 1] = rgb_buffer[3 * i + 1];
		img->imageData[3 * i + 2] = rgb_buffer[3 * i + 0];
	}
}


void
resize_image(IplImage **img, int width, int height)
{
	IplImage *resized_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvResize((*img), resized_image, CV_INTER_AREA);
	cvRelease((void**) img);
	(*img) = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvCopy(resized_image, (*img));
	cvRelease((void**) &resized_image);
}


carmen_point_t
run_cnn_inference(const carmen_localize_neural_imagepos_message &keyframe, const carmen_localize_neural_imagepos_message &curframe)
{
	carmen_point_t delta_pose = {0.0,0.0,0.0};
	IplImage *keyframe_image = cvCreateImage(cvSize(keyframe.width, keyframe.height), IPL_DEPTH_8U, 3);
	IplImage *curframe_image = cvCreateImage(cvSize(curframe.width, curframe.height), IPL_DEPTH_8U, 3);
	int input_size = 227;
	int channels = 3;

	copy_image(keyframe.image_data, keyframe_image, keyframe.width, keyframe.height);
	copy_image(curframe.image_data, curframe_image, curframe.width, curframe.height);

	resize_image(&keyframe_image, input_size, input_size);
	resize_image(&curframe_image, input_size, input_size);

	//cvSaveImage("keyframe.png", keyframe_image);
	//cvSaveImage("curframe.png", curframe_image);

	tensorflow::Tensor inputs(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,input_size,input_size,2*channels}));

	copy_tensor(inputs, keyframe_image->imageData, curframe_image->imageData, input_size, input_size, channels);

	tensorflow::Tensor keep_prob(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,1}));
	keep_prob.scalar<float>()() = 1.0;

	std::vector<tensorflow::Tensor> outputs;

	if (session != NULL)
	{
		tensorflow::Status status = session->Run({{"input", inputs},{"keep_prob", keep_prob}}, {"output"}, {}, &outputs);

		if (!status.ok())
		{
			std::cout << status.ToString() << "\n";
		}
		else
		{
			auto output = outputs[0].matrix<float>();
			delta_pose.x = output(0,0);
			delta_pose.y = output(0,1);
			delta_pose.theta = output(0,2);
		}
	}

	cvRelease((void**) &keyframe_image);
	cvRelease((void**) &curframe_image);

	return delta_pose;
}
