
#include <string>
#include <vector>
#include <boost/filesystem/operations.hpp>

#include <opencv/highgui.h>

#include <carmen/carmen_semantic_segmentation_reader.h>
#include <carmen/util_strings.h>
#include <carmen/util_io.h>

using namespace std;
using namespace cv;


SemanticSegmentationLoader::SemanticSegmentationLoader(string log_path, string data_path)
{
	vector<string> log_path_splitted = string_split(log_path, "/");
	string log_name = log_path_splitted[log_path_splitted.size() - 1];
	_log_data_dir = data_path + string("/data_") + log_name + string("/semantic");
}


SemanticSegmentationLoader::~SemanticSegmentationLoader()
{
}


cv::Mat
SemanticSegmentationLoader::load(DataSample *sample)
{
	// static to prevent reallocation.
	static char seg_img_path[256];

	sprintf(seg_img_path, "%s/%lf-r.png", _log_data_dir.c_str(), sample->image_time);

	if (!boost::filesystem::exists(seg_img_path))
		exit(printf("Segmented image '%s' not found.\n", seg_img_path));

	cv::Mat raw_img = cv::imread(seg_img_path);

	int height = (int) (0.75 * raw_img.cols);
	static Mat complete_img = Mat::ones(height, raw_img.cols, CV_8UC3) * 19;

	int top_limit = (50. / 480.) * height;
	int bottom_limit = height - (110. / 480.) * height;

	raw_img.copyTo(complete_img(Rect(0, top_limit, raw_img.cols, bottom_limit - top_limit)));

	return complete_img;
}

