#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/road_mapper.h>

#include <wordexp.h>
#include "road_mapper_utils.h"
#include <vector>
using namespace std;

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
	// As linhas abaixo sao soh para fazer generate_sample() funcionar. Esta funcao nao será necessaria no futuro.
	cv::Point pt = cv::Point(g_sample_width/2, g_sample_height/2);
	// ROI point is on the top-left corner
	cv::Rect roi = cv::Rect(cv::Point(0, 0), cv::Size(g_sample_width, g_sample_height));
	cv::Mat sample;

	cv::namedWindow("remission", cv::WINDOW_NORMAL);
	cv::moveWindow("remission", 90, 60);
	cv::namedWindow("sample", cv::WINDOW_NORMAL);
	cv::moveWindow("sample", 140, 60);

	if (g_remission_image_channels == 1 || g_remission_image_channels == '*')
	{
		g_remission_map_img = new cv::Mat(remission_map.config.y_size, remission_map.config.x_size, CV_8UC1);
		remission_map_to_image(&remission_map, g_remission_map_img, 1);
		// Chamar a rede neural várias vezes abaixo para gerar e salvar o road_map, ao inves de salvar sample.png
		generate_sample(*g_remission_map_img, pt, 0.0, roi, (char*) ("sample.png"));
	}
	if (g_remission_image_channels == 3 || g_remission_image_channels == '*')
	{
		g_remission_map_img3 = new cv::Mat(remission_map.config.y_size, remission_map.config.x_size, CV_8UC3, cv::Scalar::all(0));
		remission_map_to_image(&remission_map, g_remission_map_img3, 3);
		cv::imshow("remission", *g_remission_map_img3);
		// Chamar a rede neural várias vezes abaixo para gerar e salvar o road_map, ao inves de salvar sample.png
		int margin_x = (g_sample_width - g_sampling_stride)/2;
		int margin_y = (g_sample_height - g_sampling_stride)/2;
		for (int i = 0; i < remission_map.config.x_size; i += g_sampling_stride)
		{
			for (int j = 0; j < remission_map.config.y_size; j += g_sampling_stride)
			{
				int rect_x = i - margin_x, rect_y = j - margin_y;
				int rect_width = g_sample_width, rect_height = g_sample_height;
				sample = get_padded_roi(*g_remission_map_img3, rect_x, rect_y, rect_width, rect_height, cv::Scalar::all(255));
				cv::moveWindow("sample", 10 + g_sample_width + rect_x, 10 + rect_y);
				cv::imshow("sample", sample);
				printf("\nPress \"Esc\" key to continue...\n");
				while((cv::waitKey() & 0xff) != 27);
//				classifier.Predict(img, LUT_file);
//				update_roap_map;
			}
		}
	}
	sample.release();
	cv::destroyWindow("sample");
	cv::destroyWindow("remission");
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

	const char usage[] = "[-v]";
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
			printf("Verbose option set.\n");
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

	register_handlers();
	carmen_ipc_dispatch();

	return 0;
}
