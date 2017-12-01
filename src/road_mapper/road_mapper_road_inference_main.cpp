#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/road_mapper.h>

#include <wordexp.h>
#include "road_mapper_utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <caffe/caffe.hpp>
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace std;
using namespace caffe;  // NOLINT(build/namespaces)
using std::string;

static int g_sample_width = 0;
static int g_sample_height = 0;
static double g_distance_samples = 0.0;
static int g_n_offsets = 0;
static int g_n_rotations = 0;
static double g_distance_offset = 0.0;
char* g_out_path;
static int g_image_channels = 0;
static int g_image_class_bits = 0;
static int g_remission_image_channels = 0;
static int g_sampling_stride = 0;
char* g_prototxt_filename;
char* g_caffemodel_filename;
char* g_label_colours_filename;
wordexp_t g_out_path_p, g_prototxt_filename_p, g_caffemodel_filename_p, g_label_colours_filename_p;
int g_verbose = 0;

cv::Mat *g_road_map_img;
cv::Mat *g_road_map_img3;
cv::Mat *g_remission_map_img;
cv::Mat *g_remission_map_img3;


class Classifier
{
 public:
	  Classifier(const string& model_file,
				 const string& trained_file);

	  cv::Mat Predict(const cv::Mat& img, string LUT_file);

	  cv::Mat Visualization(cv::Mat prediction_map, string LUT_file);

 private:
	  void SetMean(const string& mean_file);

	  void WrapInputLayer(std::vector<cv::Mat>* input_channels);

	  void Preprocess(const cv::Mat& img,
					  std::vector<cv::Mat>* input_channels);

 private:
	  shared_ptr<Net<float> > net_;
	  cv::Size input_geometry_;
	  int num_channels_;

};

Classifier *g_classifier;
cv::Mat g_label_colours;

Classifier::Classifier(const string& model_file, const string& trained_file)
{
	Caffe::set_mode(Caffe::GPU);

	/* Load the network. */
	net_.reset(new Net<float>(model_file, TEST));
	net_->CopyTrainedLayersFrom(trained_file);

	CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
	CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

	Blob<float>* input_layer = net_->input_blobs()[0];
	num_channels_ = input_layer->channels();
	CHECK(num_channels_ == 3 || num_channels_ == 1) << "Input layer should have 1 or 3 channels.";
	input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
}


cv::Mat
Classifier::Visualization(cv::Mat prediction_map, string LUT_file = "")
{
	cv::cvtColor(prediction_map.clone(), prediction_map, CV_GRAY2BGR);
	cv::Mat label_colours, *label_colours_p, output_image;
	if (LUT_file == "")
	{
		label_colours_p = &g_label_colours;
	}
	else
	{
		label_colours = cv::imread(LUT_file, 1);
		label_colours_p = &label_colours;
		////  cv::cvtColor(label_colours, label_colours, CV_RGB2BGR);
	}
	LUT(prediction_map, *label_colours_p, output_image);

	if (g_verbose >= 2)
	{
		cv::imshow("prediction", output_image);
		cv::waitKey(0);
	}
	return output_image;
}


/* Wrap the input layer of the network in separate cv::Mat objects (one per channel).
 * This way we save one memcpy operation and we don't need to rely on cudaMemcpy2D.
 * The last preprocessing operation will write the separate channels directly to the input layer.
 */
void
Classifier::WrapInputLayer(std::vector<cv::Mat>* input_channels)
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
Classifier::Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels)
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


cv::Mat
Classifier::Predict(const cv::Mat& img, string LUT_file = "")
{
	Blob<float>* input_layer = net_->input_blobs()[0];
	input_layer->Reshape(1, num_channels_, input_geometry_.height, input_geometry_.width);
	/* Forward dimension change to all layers. */
	net_->Reshape();

	std::vector<cv::Mat> input_channels;
	WrapInputLayer(&input_channels);

	Preprocess(img, &input_channels);

	//	struct timeval time;
	//	gettimeofday(&time, NULL); // Start Time
	//	long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
	//std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now(); //Just for time measurement

	net_->Forward();

	//	gettimeofday(&time, NULL);  //END-TIME
	//	totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);
	//	std::cout << "Processing time = " << totalTime << " ms" << std::endl;

	//std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
	//std::cout << "Processing time = " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1000000.0 << " sec" <<std::endl; //Just for time measurement

	/* Copy the output layer to a std::vector */
	Blob<float>* output_layer = net_->output_blobs()[0];

	int width = output_layer->width();
	int height = output_layer->height();
	int channels = output_layer->channels();
	//	int num = output_layer->num();

	//	std::cout << "output_blob(n,c,h,w) = " << num << ", " << channels << ", " << height << ", " << width << std::endl;

	// compute argmax
	cv::Mat class_each_row(channels, width * height, CV_32FC1, const_cast<float *>(output_layer->cpu_data()));
	class_each_row = class_each_row.t(); // transpose to make each row with all probabilities
	cv::Point maxId;    // point [x,y] values for index of max
	double maxValue;    // the holy max value itself
	cv::Mat prediction_map(height, width, CV_8UC1);
	for (int i = 0; i < class_each_row.rows; i++)
	{
		minMaxLoc(class_each_row.row(i), 0, &maxValue, 0, &maxId);
		prediction_map.at<uchar>(i) = maxId.x;
	}
	if (g_verbose >= 2)
	{
		Visualization(prediction_map, LUT_file);
	}
	return prediction_map;
}


cv::Mat
get_padded_roi(const cv::Mat &input, int top_left_x, int top_left_y, int width, int height, cv::Scalar paddingColor)
{
    int bottom_right_x = top_left_x + width;
    int bottom_right_y = top_left_y + height;

    cv::Mat output;
    if (top_left_x < 0 || top_left_y < 0 || bottom_right_x > input.cols || bottom_right_y > input.rows)
    {
        // border padding will be required
        int border_left = 0, border_right = 0, border_top = 0, border_bottom = 0;

        if (top_left_x < 0)
        {
            width = width + top_left_x;
            border_left = -1 * top_left_x;
            top_left_x = 0;
        }
        if (top_left_y < 0)
        {
            height = height + top_left_y;
            border_top = -1 * top_left_y;
            top_left_y = 0;
        }
        if (bottom_right_x > input.cols)
        {
            width = width - (bottom_right_x - input.cols);
            border_right = bottom_right_x - input.cols;
        }
        if (bottom_right_y > input.rows)
        {
            height = height - (bottom_right_y - input.rows);
            border_bottom = bottom_right_y - input.rows;
        }

        cv::Rect R(top_left_x, top_left_y, width, height);
        cv::copyMakeBorder(input(R), output, border_top, border_bottom, border_left, border_right, cv::BORDER_CONSTANT, paddingColor);
    }
    else
    {
        // no border padding required
        cv::Rect R(top_left_x, top_left_y, width, height);
        output = input(R);
    }
    return output;
}


void
generate_sample(cv::Mat map_img, cv::Point center, double angle, cv::Rect roi, char* path)
{
	cv::Mat rot_img = rotate(map_img, center, angle);
	cv::Mat sample = rot_img(roi);
	cv::imwrite(path, sample);
	rot_img.release();
	sample.release();
}


void
generate_road_map_via_deep_learning_inference(carmen_map_t remission_map)
{
	cv::Mat *remission_map_img, sample, prediction, road_map_img;
	static carmen_map_t road_map;
	static bool first_time = true;
	road_prob cell;
	double *cell_value;
	char map_path[] = ".";

	if (first_time)
	{
		carmen_grid_mapping_initialize_map(&road_map, remission_map.config.x_size, remission_map.config.resolution, 'r');
		first_time = false;
	}
	memcpy(&road_map.config, &remission_map.config, sizeof(remission_map.config));

	if (g_remission_image_channels == 1 || g_remission_image_channels == '*')
	{
		g_remission_map_img = new cv::Mat(remission_map.config.y_size, remission_map.config.x_size, CV_8UC1);
		remission_map_img = g_remission_map_img;
		remission_map_to_image(&remission_map, remission_map_img, 1);
		// Repeat the procedure below
	}
	if (g_remission_image_channels == 3 || g_remission_image_channels == '*')
	{
		g_remission_map_img3 = new cv::Mat(remission_map.config.y_size, remission_map.config.x_size, CV_8UC3);
		remission_map_img = g_remission_map_img3;
		remission_map_to_image(&remission_map, remission_map_img, 3);
		if (g_verbose >= 1)
		{
			cv::imshow("remission", *g_remission_map_img3);
		}
		int margin_x = (g_sample_width - g_sampling_stride)/2;
		int margin_y = (g_sample_height - g_sampling_stride)/2;
		for (int i = 0; i < remission_map.config.x_size; i += g_sampling_stride)
		{
			for (int j = 0; j < remission_map.config.y_size; j += g_sampling_stride)
			{
				int rect_x = i - margin_x, rect_y = j - margin_y;
				int rect_width = g_sample_width, rect_height = g_sample_height;
				sample = get_padded_roi(*g_remission_map_img3, rect_x, rect_y, rect_width, rect_height, cv::Scalar::all(255));
				if (g_verbose >= 2)
				{
					cv::moveWindow("sample", rect_x, rect_y);
					cv::imshow("sample", sample);
				}
				prediction = g_classifier->Predict(sample);
				if (g_verbose >= 2)
				{
					printf("\nPress \"Esc\" key to continue...\n");
					while((cv::waitKey() & 0xff) != 27);
				}
				// Update inferred roap map, using only a central square (stride x stride)
				for (int x = margin_x; x < (margin_x + g_sampling_stride); x++)
				{
					for (int y = margin_y; y < (margin_y + g_sampling_stride); y++)
					{
						road_mapper_cell_class_to_prob(&cell, prediction.at<uchar>(x, y), g_image_class_bits);
						cell_value = (double*) &cell;
						road_map.map[i + x - margin_x][j + y - margin_y] = *cell_value;
					}
				}
			}
		}
		// Save complete road map
		bool ok = (carmen_grid_mapping_save_block_map_by_origin(map_path, 'r', &road_map) != 0);
		if (!ok)
		{
			printf("ERROR: Could not save road map: (%.0lf,%.0lf) at %s\n", road_map.config.x_origin, road_map.config.y_origin, map_path);
		}

		if (g_verbose >= 1)
		{
			if (ok)
			{
				printf("Road map saved: (%.0lf,%.0lf) at %s\n", road_map.config.x_origin, road_map.config.y_origin, map_path);
			}
			road_map_to_image(&road_map, &road_map_img);
			cv::imshow("road map", road_map_img);
			printf("\nPress \"Esc\" key to continue...\n");
			while((cv::waitKey() & 0xff) != 27);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_map_handler(carmen_map_server_localize_map_message *msg)
{
	static carmen_map_t remission_map;
	static bool first_time = true;

	if (first_time)
	{
		carmen_grid_mapping_initialize_map(&remission_map, msg->config.x_size, msg->config.resolution, 'm');
		first_time = false;
	}

	memcpy(remission_map.complete_map, msg->complete_mean_remission_map, sizeof(double) * msg->size);
	remission_map.config = msg->config;

	generate_road_map_via_deep_learning_inference(remission_map);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("road_mapper_road_inference: disconnected.\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
read_parameters(int argc, char **argv)
{
	char *out_path = (char *)".";
	char *prototxt_filename = (char *)".";
	char *caffemodel_filename = (char *)".";
	char *label_colours_filename = (char *)".";
	char *image_channels = (char *)"*";
	char *remission_image_channels = (char *)"*";
	carmen_param_t param_list[] =
	{
			{(char*)"road_mapper",  (char*)"sample_width", 				CARMEN_PARAM_INT, 		&(g_sample_width), 				0, NULL},
			{(char*)"road_mapper",  (char*)"sample_height",				CARMEN_PARAM_INT, 		&(g_sample_height), 			0, NULL},
			{(char*)"road_mapper",  (char*)"distance_sample",			CARMEN_PARAM_DOUBLE, 	&(g_distance_samples), 			0, NULL},
			{(char*)"road_mapper",  (char*)"n_offset",					CARMEN_PARAM_INT, 		&(g_n_offsets), 				0, NULL},
			{(char*)"road_mapper",  (char*)"n_rotation",				CARMEN_PARAM_INT, 		&(g_n_rotations), 				0, NULL},
			{(char*)"road_mapper",  (char*)"distance_offset",			CARMEN_PARAM_DOUBLE, 	&(g_distance_offset),			0, NULL},
			{(char*)"road_mapper",  (char*)"out_path",					CARMEN_PARAM_STRING, 	&(out_path),					0, NULL},
			{(char*)"road_mapper",  (char*)"image_channels",			CARMEN_PARAM_STRING, 	&(image_channels),				0, NULL},
			{(char*)"road_mapper",  (char*)"image_class_bits",			CARMEN_PARAM_INT, 		&(g_image_class_bits),			0, NULL},
			{(char*)"road_mapper",  (char*)"remission_image_channels",	CARMEN_PARAM_STRING, 	&(remission_image_channels),	0, NULL},
			{(char*)"road_mapper",  (char*)"sampling_stride",			CARMEN_PARAM_INT, 		&(g_sampling_stride), 			0, NULL},
			{(char*)"road_mapper",  (char*)"prototxt_filename",			CARMEN_PARAM_STRING, 	&(prototxt_filename),			0, NULL},
			{(char*)"road_mapper",  (char*)"caffemodel_filename",		CARMEN_PARAM_STRING, 	&(caffemodel_filename),			0, NULL},
			{(char*)"road_mapper",  (char*)"label_colours_filename",	CARMEN_PARAM_STRING, 	&(label_colours_filename),		0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	// expand environment variables on path to full path
	wordexp(out_path, &g_out_path_p, 0 );
	g_out_path = *g_out_path_p.we_wordv;
	wordexp(prototxt_filename, &g_prototxt_filename_p, 0 );
	g_prototxt_filename = *g_prototxt_filename_p.we_wordv;
	wordexp(caffemodel_filename, &g_caffemodel_filename_p, 0 );
	g_caffemodel_filename = *g_caffemodel_filename_p.we_wordv;
	wordexp(label_colours_filename, &g_label_colours_filename_p, 0 );
	g_label_colours_filename = *g_label_colours_filename_p.we_wordv;

	// image channels
	g_image_channels = '*';
	if(strcmp(image_channels, "1") == 0 || strcmp(image_channels, "3") == 0)
	{
		g_image_channels = atoi(image_channels);
	}
	g_remission_image_channels = '*';
	if(strcmp(remission_image_channels, "1") == 0 || strcmp(remission_image_channels, "3") == 0)
	{
		g_remission_image_channels = atoi(remission_image_channels);
	}

	const char usage[] = "[-v [<level>]]";
	for(int i = 1; i < argc; i++)
	{
		if(strncmp(argv[i], "-h", 2) == 0 || strncmp(argv[i], "--help", 6) == 0)
		{
			printf("Usage:\n%s %s\n", argv[0], usage);
			exit(1);
		}
		else if(strncmp(argv[i], "-v", 2) == 0 || strncmp(argv[i], "--verbose", 9) == 0)
		{
			g_verbose = 1;
			if ((i + 1) < argc && atoi(argv[i + 1]) > 0)
			{
				g_verbose = atoi(argv[i + 1]);
				i++;
			}
			printf("Verbose option set to level %d.\n", g_verbose);
		}
		else
		{
			printf("Ignored command line parameter: %s\n", argv[i]);
			printf("Usage:\n%s %s\n", argv[0], usage);
		}
	}
}


static void
define_messages()
{
	carmen_map_server_define_localize_map_message();
}


static void
register_handlers()
{
	carmen_map_server_subscribe_localize_map_message(NULL, (carmen_handler_t) localize_map_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);
	define_messages();

	signal(SIGINT, shutdown_module);

	::google::InitGoogleLogging(argv[0]);

	g_classifier = new Classifier(g_prototxt_filename, g_caffemodel_filename);
	g_label_colours = cv::imread(g_label_colours_filename, 1);

	register_handlers();
	carmen_ipc_dispatch();

	return 0;
}
