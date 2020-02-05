// opencv stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp>

// c++ stuff
#include <chrono>
#include <iomanip>  // for setfill
#include <iostream>
#include <string>


// net stuff
#include "selector.hpp"
namespace cl = rangenet::segmentation;

// standalone lib h
#include "rangenet_inference.h"

// boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

typedef std::tuple< u_char, u_char, u_char> color;

//global variables
std::unique_ptr<cl::Net> net;

int *
rangenet_process_point_cloud(std::vector<float> values, uint32_t num_points, std::string scan)
{
	// predict each image
	std::cout << "RangeNet: Predicting image: " << scan << std::endl;

	// predict
	std::vector<std::vector<float>> semantic_scan = net->infer(values, num_points);

	//Mounting return vector for mapper_main
	int * vel_rangenet = (int *) malloc (sizeof(int) * num_points);
	std::vector<float> labels_prob;
	labels_prob.resize(num_points);
	for (uint32_t i = 0; i < num_points; ++i) {
		labels_prob[i] = 0;
		for (int32_t j = 0; j < 20; j++)
		{
			if (labels_prob[i] <= semantic_scan[i][j])
			{
				vel_rangenet[i] = j;
				labels_prob[i] = semantic_scan[i][j];
			}
		}
	}
	
	std::cout << "RangeNet: Inference " << scan << " finished! "<< std::endl;

	return vel_rangenet;
}

void
rangenet_infer(std::vector<float> values, uint32_t num_points, std::string scan)
{
	bool verbose = true;
	// predict each image
	std::cout << std::setfill('=') << std::setw(80) << "" << std::endl;
	std::cout << "Predicting image: " << scan << std::endl;

	// predict
	std::vector<std::vector<float>> semantic_scan = net->infer(values, num_points);

	// get point cloud
	std::vector<cv::Vec3f> points = net->getPoints(values, num_points);

	// get color mask
	std::vector<cv::Vec3b> color_mask = net->getLabels(semantic_scan, num_points);

	// print the output
	if (verbose) {
		cv::viz::Viz3d window = cv::viz::getWindowByName("semantic scan");
		cv::viz::WCloud cloudWidget(points, color_mask);
		while (!window.wasStopped()) {
			window.showWidget("cloud", cloudWidget);
			window.spinOnce(30, true);
			//cv::waitKey(30);
		}
	}
	std::cout << std::setfill('=') << std::setw(80) << "" << std::endl;

	std::cout << "Example finished! "<< std::endl;

}

void
initialize_rangenet()
{
	bool verbose = false;
	std::string npath = "/sharedlib/rangenet_lib/model/darknet53/";
	std::string stpath (std::getenv("CARMEN_HOME"));
	std::string path = stpath + npath;
	std::string backend = "tensorrt";

	// create a network
	net = cl::make_net(path, backend);

	// set verbosity
	net->verbosity(verbose);
}
