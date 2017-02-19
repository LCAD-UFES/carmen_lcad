#include <carmen/carmen.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#if CV_MAJOR_VERSION == 2
#include <opencv2/contrib/detection_based_tracker.hpp>
#endif
#include <stdio.h>
#include <dlib/svm.h>

#define WIDTH 9
#define HEIGHT 20

using namespace dlib;
using namespace cv;
using namespace std;

//Parameters

//SVM
typedef matrix<double, 180, 1> sample_type;
typedef radial_basis_kernel<sample_type> kernel_type;
typedef decision_function<kernel_type> dec_funct_type;
typedef normalized_function<dec_funct_type> funct_type;
funct_type trained_svm;
string svm_train_name = getenv("CARMEN_HOME")+ (string) "/data/traffic_light/svm.dat";

//Carmen
static int camera;
static int image_width;
static int image_height;
static int infinite = 9999.0;

//Haar Cascade
string ts_cascade_name = getenv("CARMEN_HOME")+ (string) "/data/traffic_light/data.xml";
CascadeClassifier ts_cascade;

//Messages
carmen_mapping_traffic_light_message mapping_traffic_light_message;

#define MAX_TRAFFIC_LIGHTS_IN_IMAGE 10
static int num = 0;
static carmen_traffic_light traffic_lights_detected[MAX_TRAFFIC_LIGHTS_IN_IMAGE];


sample_type
get_traffic_light_image_in_svm_format(cv::Mat frame, CvPoint p1, CvPoint p2)
{
	Rect myROI(p1, p2);
	cv::Mat croppedImage;
	cv::Mat(frame, myROI).copyTo(croppedImage);

    cv::Mat resized_image;
    resized_image.create(HEIGHT, WIDTH, CV_8UC3);
    resize(croppedImage, resized_image, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_CUBIC);

    std::vector<cv::Mat> channels;
    split(resized_image, channels);

    cv::Mat aux;
    aux = ((channels.at(0) - channels.at(1)) + 255) / 2;

    sample_type traffic_light_image_in_svm_format;
    for (int x = 0; x < resized_image.rows; x++)
        for (int y = 0; y < resized_image.cols; y++)
        	traffic_light_image_in_svm_format(x * WIDTH + y) = (double) (aux.at<uchar>(x, y));

    return (traffic_light_image_in_svm_format);
}


std::vector<Rect>
detect_traffic_lights(const cv::Mat frame)
{
	Rect ROI(Point(image_width / 4, 0), Point(image_width / 4 * 3, image_height / 2));
	cv::Mat half_image;
	cv::Mat(frame, ROI).copyTo(half_image);

//	cv::cvtColor(half_image, half_image, CV_BGR2RGB);
//	namedWindow("Display window", WINDOW_AUTOSIZE);
//	imshow("Display window", half_image);
//	waitKey(1);

	cv::Mat frame_gray;
	cvtColor(half_image, frame_gray, CV_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

//	namedWindow("Display window", WINDOW_AUTOSIZE);
//	imshow("Display window", frame_gray);
//	waitKey(1);

	//-- Detect traffic lights
	std::vector<Rect> semaphores;
	ts_cascade.detectMultiScale(frame_gray, semaphores, 1.03, 3, 0, Size(5, 10), Size(200, 400));

	return (semaphores);
}


void
add_traffic_light_to_message(carmen_traffic_light_message *traffic_light_message, int color, CvPoint p1, CvPoint p2, size_t i)
{
	carmen_traffic_light traffic_light;

	traffic_light.color = color;
	traffic_light.x1 = p1.x;
	traffic_light.y1 = p1.y;
	traffic_light.x2 = p2.x;
	traffic_light.y2 = p2.y;
	traffic_light_message->traffic_lights[i] = traffic_light;

	// @@@ Alberto: pra que o codigo abaixo?
	num++;
	ofstream out;
	out.open("saida.txt", std::fstream::out | std::fstream::app);
	if (traffic_light.color == TRAFFIC_LIGHT_RED)
		out << traffic_light_message->distance << " " << num << " 1 0" << endl;
	else if (traffic_light.color == TRAFFIC_LIGHT_GREEN)
		out << traffic_light_message->distance << " " << num << " 0 1" << endl;
	// @@@ Alberto: e o amarelo?
	out.close();
}


cv::Mat
detect_traffic_lights_and_recognize_their_state(carmen_traffic_light_message *traffic_light_message,
		carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
    cv::Mat frame(image_height, image_width, CV_8UC3);
	memcpy(frame.data, stereo_image->raw_right, stereo_image->image_size);

//	cv::cvtColor(frame, frame, CV_BGR2RGB);
//	namedWindow("Display window", WINDOW_AUTOSIZE);
//	imshow("Display window", frame);
//	waitKey(1);

	std::vector<Rect> traffic_light_rectangles = detect_traffic_lights(frame);

	if (traffic_light_message->distance < 200.0 && traffic_light_message->distance != -1.0)
	{
		traffic_light_message->num_traffic_lights = traffic_light_rectangles.size();

		for (size_t i = 0; i < traffic_light_rectangles.size() && i < MAX_TRAFFIC_LIGHTS_IN_IMAGE; i++)
		{
			CvPoint p1, p2;
			p1.x = traffic_light_rectangles[i].x + image_width / 4;
			p1.y = traffic_light_rectangles[i].y;
			p2.x = p1.x + traffic_light_rectangles[i].width;
			p2.y = p1.y + traffic_light_rectangles[i].height;

			sample_type traffic_light_image_in_svm_format = get_traffic_light_image_in_svm_format(frame, p1, p2);

			if (trained_svm(traffic_light_image_in_svm_format) >= 0)
				add_traffic_light_to_message(traffic_light_message, TRAFFIC_LIGHT_RED, p1, p2, i);
			else if (trained_svm(traffic_light_image_in_svm_format) < 0)
				add_traffic_light_to_message(traffic_light_message, TRAFFIC_LIGHT_GREEN, p1, p2, i);
			// @@@ Alberto: e o amarelo?
		}
	}

	return (frame);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
publish_traffic_lights(carmen_traffic_light_message *traffic_light_message)
{
    carmen_traffic_light_publish_message(camera, traffic_light_message);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_mapping_traffic_light_message_handler(carmen_mapping_traffic_light_message *msg)
{
    mapping_traffic_light_message = *msg;
}


void
carmen_bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	carmen_traffic_light_message traffic_light_message;
	cv::Mat image;

    traffic_light_message.traffic_light_image_size = image_height * image_width * 3;
    traffic_light_message.traffic_lights = traffic_lights_detected;
    traffic_light_message.host = carmen_get_host();

	if (mapping_traffic_light_message.distance < 200.0 && mapping_traffic_light_message.distance != -1.0)
	{
        traffic_light_message.distance = mapping_traffic_light_message.distance;

        image = detect_traffic_lights_and_recognize_their_state(&traffic_light_message, stereo_image);

        traffic_light_message.traffic_light_image = image.data;
        traffic_light_message.timestamp = stereo_image->timestamp;
    }
    else
    {
        traffic_light_message.distance = infinite;
        traffic_light_message.num_traffic_lights = 0;
        traffic_light_message.traffic_light_image = stereo_image->raw_right;
        traffic_light_message.timestamp = stereo_image->timestamp;
    }

    publish_traffic_lights(&traffic_light_message);
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
traffic_light_algorithm_initialization()
{
	memset(&mapping_traffic_light_message, 0, sizeof(mapping_traffic_light_message));
	mapping_traffic_light_message.distance = -1.0;

    //Read the haar cascade trained
    ts_cascade.load(ts_cascade_name);

    //Read the svm trained
    ifstream fin(svm_train_name.c_str(), ios::binary);
    deserialize(trained_svm, fin);
}


static int
read_parameters(int argc, char **argv)
{
    int num_items;
    char bumblebee_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

    carmen_param_t param_list[] =
    {
        { bumblebee_string, (char*) "width", CARMEN_PARAM_INT, &image_width, 0, NULL},
        { bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &image_height, 0, NULL}
    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return (0);
}


void
subscribe_camera_mapping_traffic_light_messages()
{
    carmen_mapping_traffic_light_subscribe(NULL, (carmen_handler_t) carmen_mapping_traffic_light_message_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler,
    		CARMEN_SUBSCRIBE_LATEST);
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    if ((argc != 2))
        carmen_die("%s: Wrong number of parameters. Traffic Light requires 1 parameters and received %d parameter(s). \n"
        		   "Usage:\n %s <camera_number>\n", argv[0], argc - 1, argv[0]);

    camera = atoi(argv[1]);

    read_parameters(argc, argv);
    carmen_traffic_light_define_messages(camera);
    subscribe_camera_mapping_traffic_light_messages();

    traffic_light_algorithm_initialization();

    carmen_ipc_dispatch();

    return (EXIT_SUCCESS);
}
