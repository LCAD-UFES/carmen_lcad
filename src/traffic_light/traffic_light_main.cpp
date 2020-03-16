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


//USE_CAFFE IS DEFINED IN MAKEFILE IF CAFFE_HOME ENV. VARIABLE IS SET
#ifdef USE_CAFFE
#include "squeezenet_tlight.h"
#endif

#include <stdio.h>

#ifdef USE_DLIB
#include <dlib/svm.h>

using namespace dlib;
#include <carmen/tlight_state_recog.h>
#include <carmen/tlight_factory.h>
#endif

int use_squeezenet = 1;
int gpu_mode = 1;
int gpu_device_id = 0;

//CNN to recognize traffic_light given an image

#ifdef USE_CAFFE
SqueezeNet* squeezenet_classify = NULL;
#endif

#define WIDTH 9
#define HEIGHT 20
const int USE_VGRAM = 1;

using namespace cv;
using namespace std;
string str_prototxt = getenv("CARMEN_HOME")+ (string) "/data/traffic_light/squeezenet/deploy.prototxt";
string str_caffemodel = getenv("CARMEN_HOME")+ (string) "/data/traffic_light/squeezenet/train_squeezenet_trainval_manual_p2__iter_3817.caffemodel";

//Parameters

#ifdef USE_DLIB
//SVM
typedef matrix<double, 180, 1> sample_type;
typedef radial_basis_kernel<sample_type> kernel_type;
typedef decision_function<kernel_type> dec_funct_type;
typedef normalized_function<dec_funct_type> funct_type;
funct_type trained_svm;
string svm_train_name = getenv("CARMEN_HOME")+ (string) "/data/traffic_light/svm.dat";

TLightRecogInterface *recognizer;
#endif

//Camera
static int camera;
static int image_width;
static int image_height;

//Haar Cascade
string ts_cascade_name = getenv("CARMEN_HOME")+ (string) "/data/traffic_light/data.xml";
CascadeClassifier ts_cascade;

// Dection infrastructure
#define MAX_TRAFFIC_LIGHTS_IN_IMAGE 10
// Ver valores abaixo no arquivo height_in_pixels_x_distance.ods

static double dist_correction = 20.0;
static double focal_distance = 0;
static int roi_x = 0;
static int roi_y = 0;
static int roi_w = 0;
static int roi_h = 0;

static carmen_traffic_light traffic_lights_detected[MAX_TRAFFIC_LIGHTS_IN_IMAGE];

// Localization infrastructure
carmen_localize_ackerman_globalpos_message last_localize_message;
static bool has_localize_message = false;

// Annotations infrastructure
carmen_rddf_annotation_message last_annotation_message;
static double infinite = 9999.0;

// Database generation infrastructure
static int generate_database = 0;
static char *database_path;
carmen_vector_3D_t nearest_traffic_light_pose;


double
compute_distance_to_the_traffic_light()
{
    double nearest_traffic_light_distance = infinite;

    for (int i = 0; i < last_annotation_message.num_annotations; i++)
    {
    	if (last_annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT)
    	{
    		double distance = sqrt(pow(last_localize_message.globalpos.x - last_annotation_message.annotations[i].annotation_point.x, 2) +
							pow(last_localize_message.globalpos.y - last_annotation_message.annotations[i].annotation_point.y, 2));
    		if (distance < nearest_traffic_light_distance)
    		{
    			bool orientation_ok = fabs(carmen_radians_to_degrees(last_localize_message.globalpos.theta - last_annotation_message.annotations[i].annotation_orientation)) < 10.0 ? 1 : 0;
    			bool behind = fabs(carmen_normalize_theta((atan2(last_localize_message.globalpos.y - last_annotation_message.annotations[i].annotation_point.y,
															last_localize_message.globalpos.x - last_annotation_message.annotations[i].annotation_point.x) - M_PI -
													 	 	last_localize_message.globalpos.theta))) > M_PI_2;

				if (distance <= MAX_TRAFFIC_LIGHT_DISTANCE && orientation_ok && behind == false)
				{
	    			nearest_traffic_light_distance = distance;
	    			nearest_traffic_light_pose = last_annotation_message.annotations[i].annotation_point;
				}
    		}
    	}
    }

    return (nearest_traffic_light_distance);
}

#ifdef USE_DLIB
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
#endif

std::vector<Rect>
detect_traffic_lights(const cv::Mat frame)
{
	// Parametros da Bumblebee
	Rect ROI(roi_x, roi_y, roi_w, roi_h);
	cv::Mat half_image;
	cv::Mat(frame, ROI).copyTo(half_image);

	cv::Mat frame_gray;
	cvtColor(half_image, frame_gray, CV_BGR2GRAY);

//	Descomente para visualizar o ROI
//	Mat bola = frame.clone();
//	cv::rectangle(bola, ROI, Scalar(0,0,255), 6);
//	Mat res(Size(bola.cols / 2, bola.rows / 2), bola.type());
//	resize(bola, res, res.size());
//	imshow("Display window", res);
//	waitKey(1);

	//-- Detect traffic lights
	std::vector<Rect> semaphores;
	ts_cascade.detectMultiScale(frame_gray, semaphores, 1.05, 3, 0, Size(5, 12), Size(60, 150));

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
}


int
map_net_class_to_annotation_code(int prediction)
{
	/**annotation_code | Net_Class
	 * 	red = 0,			1
	 * 	green = 1			2
	 * 	yellow = 2			0 none
	 * 	off = 3				0 none
	 */
	int label;
	if (prediction == 0)
		label = 3;
	else if (prediction == 1)
		label = 0;
	else
		label = 1;
	return label;
}

void
detect_traffic_lights_and_recognize_their_state(carmen_traffic_light_message *traffic_light_message,
		carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
    cv::Mat frame(image_height, image_width, CV_8UC3);
	memcpy(frame.data, stereo_image->raw_right, stereo_image->image_size);

	if (USE_VGRAM)
		cv::cvtColor(frame, frame, CV_BGR2RGB);

	if (traffic_light_message->traffic_light_annotation_distance < MAX_TRAFFIC_LIGHT_DISTANCE &&
			traffic_light_message->traffic_light_annotation_distance != -1.0)
	{
		int num_traffic_lights_accepted;

#ifdef USE_CAFFE
		if (use_squeezenet)
		{
			// Parametros da Bumblebee
			Rect ROI(roi_x, roi_y, roi_w, roi_h);
			cv::Mat half_image;
			cv::Mat(frame, ROI).copyTo(half_image);

			int net_prediction = squeezenet_classify->Predict(half_image);
			//			cv::imshow("Half_frame",half_image);
			//			waitKey(1);
			int label = map_net_class_to_annotation_code(net_prediction);
			int traffic_light_status = label + RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED;

			//Just to keep things working - this mode does not detect position
			CvPoint p1, p2;
			p1.x = 0; p1.y = 1;
			p2.x = 1; p2.y = 2;

			add_traffic_light_to_message(traffic_light_message, traffic_light_status, p1, p2, 0);
			num_traffic_lights_accepted = 1;
		}
		else
#endif
		{
			// Traffic lights detection
			num_traffic_lights_accepted = 0;
			std::vector<Rect> traffic_light_rectangles = detect_traffic_lights(frame);

			double expected_traffic_light_height = 1.0 * focal_distance / (traffic_light_message->traffic_light_annotation_distance + dist_correction);
			for (size_t i = 0; i < traffic_light_rectangles.size() && i < MAX_TRAFFIC_LIGHTS_IN_IMAGE; i++)
			{
				//			printf("%lf %d\n", traffic_light_message->traffic_light_annotation_distance, traffic_light_rectangles[i].height);
				double percentual_difference = fabs(1.0 - traffic_light_rectangles[i].height / expected_traffic_light_height);
				if (percentual_difference < 0.55)
				{
					CvPoint p1, p2;
					p1.x = traffic_light_rectangles[i].x + roi_x;
					p1.y = traffic_light_rectangles[i].y + roi_y;
					p2.x = p1.x + traffic_light_rectangles[i].width;
					p2.y = p1.y + traffic_light_rectangles[i].height;
#ifdef USE_DLIB
					// FILIPE
					if (USE_VGRAM)
					{
						int label = 0;

						Rect r = traffic_light_rectangles[i];
						r.x += roi_x;
						r.y += roi_y;

						label = recognizer->run(frame, r);
						int traffic_light_status = label + RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED;
						add_traffic_light_to_message(traffic_light_message, traffic_light_status, p1, p2, num_traffic_lights_accepted);
					}
					else
					{
						sample_type traffic_light_image_in_svm_format = get_traffic_light_image_in_svm_format(frame, p1, p2);

						// Traffic lights state recognition
						if (trained_svm(traffic_light_image_in_svm_format) >= 0)
							add_traffic_light_to_message(traffic_light_message, RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED, p1, p2, num_traffic_lights_accepted);
						else if (trained_svm(traffic_light_image_in_svm_format) < 0)
							add_traffic_light_to_message(traffic_light_message, RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN, p1, p2, num_traffic_lights_accepted);
						// @@@ Alberto: E o amarelo? E a rejeicao de deteccoes?
					}
#endif

					num_traffic_lights_accepted++;
				}
			}
		}
		traffic_light_message->num_traffic_lights = num_traffic_lights_accepted;
	}
}


Mat*
bumblebee_to_opencv(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	static Mat *m = NULL;

	// constructing Mat objects is really time consuming, so I alloc it just once.
	if (m == NULL)
		m = new cv::Mat(image_height, image_width, CV_8UC3);

	memcpy(m->data, stereo_image->raw_right, stereo_image->image_size);
	cv::cvtColor(*m, *m, CV_BGR2RGB);

	return m;
}


void
save_image_and_detections(carmen_traffic_light_message traffic_light_message, carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	static int counter = 0;

	if (traffic_light_message.num_traffic_lights == 0)
	{
		const char *filename = "undetected.txt";
		static char image_name[1024];
		static char file_name_with_dir[2048];

		sprintf(image_name, "%s/img/image_%04d_%lf_%lf_%lf.png",
				database_path, counter, stereo_image->timestamp,
				nearest_traffic_light_pose.x, nearest_traffic_light_pose.y);

		sprintf(file_name_with_dir, "%s/%s", database_path, filename);

		FILE *database_file = fopen(file_name_with_dir, "a");

		if (database_file == NULL)
			exit(printf("Error: Unable to open the output file '%s'\n", file_name_with_dir));

		Mat *m = bumblebee_to_opencv(stereo_image);
		imwrite(image_name, *m);

		fprintf(database_file, "%s\n", image_name);

		fclose(database_file);
	}

	else
	{
		for (int i = 0; i < traffic_light_message.num_traffic_lights; i ++)
		{
			const char *filename;

			if (traffic_light_message.traffic_lights[i].color == RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED)
				filename = "red.txt";
			else if (traffic_light_message.traffic_lights[i].color == RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN)
				filename = "green.txt";
			else if (traffic_light_message.traffic_lights[i].color == RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_YELLOW)
				filename = "yellow.txt";
			else if (traffic_light_message.traffic_lights[i].color == RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF)
				filename = "off.txt";

			static char image_name[1024];
			static char file_name_with_dir[2048];

			sprintf(image_name, "%s/img/image_%04d_%lf_%lf_%lf.png",
					database_path, counter, stereo_image->timestamp,
					nearest_traffic_light_pose.x, nearest_traffic_light_pose.y);

			sprintf(file_name_with_dir, "%s/%s", database_path, filename);

			FILE *database_file = fopen(file_name_with_dir, "a");

			if (database_file == NULL)
				exit(printf("Error: Unable to open the output file '%s'\n", file_name_with_dir));

			Mat *m = bumblebee_to_opencv(stereo_image);
			imwrite(image_name, *m);

			fprintf(database_file, "%s %d %d %d %d\n", image_name,
					traffic_light_message.traffic_lights[i].x1,
					traffic_light_message.traffic_lights[i].y1,
					traffic_light_message.traffic_lights[i].x2,
					traffic_light_message.traffic_lights[i].y2);

			fclose(database_file);
		}
	}
	counter++;
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
carmen_bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	carmen_traffic_light_message traffic_light_message;
	cv::Mat image;
	double nearest_traffic_light_distance = infinite;

	if (has_localize_message)
		nearest_traffic_light_distance = compute_distance_to_the_traffic_light();

    traffic_light_message.traffic_lights = traffic_lights_detected;
    traffic_light_message.host = carmen_get_host();

	if (nearest_traffic_light_distance < MAX_TRAFFIC_LIGHT_DISTANCE && nearest_traffic_light_distance != -1.0)
	{
        traffic_light_message.traffic_light_annotation_distance = nearest_traffic_light_distance;
        detect_traffic_lights_and_recognize_their_state(&traffic_light_message, stereo_image);

        if (generate_database)
        	save_image_and_detections(traffic_light_message, stereo_image);

        traffic_light_message.timestamp = stereo_image->timestamp;
    }
    else
    {
        traffic_light_message.traffic_light_annotation_distance = infinite;
        traffic_light_message.num_traffic_lights = 0;
        traffic_light_message.timestamp = stereo_image->timestamp;
    }

    publish_traffic_lights(&traffic_light_message);
}


void
carmen_rddf_annotation_message_handler(carmen_rddf_annotation_message *msg)
{
	last_annotation_message = *msg;
}


void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    last_localize_message = *msg;
    has_localize_message = true;
}


static void
shutdown_traffic_light(int x)
{
    if (x == SIGINT)
    {
#ifdef USE_CAFFE
    	if (squeezenet_classify != NULL)
    		delete squeezenet_classify;
#endif
        carmen_verbose("Disconnecting Traffic Light.\n");
        carmen_ipc_disconnect();

        exit(1);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
traffic_light_module_initialization()
{
    //Read the haar cascade trained
    ts_cascade.load(ts_cascade_name);
    //Read the svm trained
#ifdef USE_DLIB
    ifstream fin(svm_train_name.c_str(), ios::binary);
    deserialize(trained_svm, fin);

    recognizer = TLightStateRecogFactory::build("mlp");
#endif

#ifdef USE_CAFFE
    if (use_squeezenet)
    	squeezenet_classify = new SqueezeNet(str_prototxt, str_caffemodel, gpu_mode, gpu_device_id);
#endif

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
        { bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &image_height, 0, NULL},
        { bumblebee_string, (char*) "tlight_roi_x", CARMEN_PARAM_INT, &roi_x, 0, NULL},
        { bumblebee_string, (char*) "tlight_roi_y", CARMEN_PARAM_INT, &roi_y, 0, NULL},
        { bumblebee_string, (char*) "tlight_roi_w", CARMEN_PARAM_INT, &roi_w, 0, NULL},
        { bumblebee_string, (char*) "tlight_roi_h", CARMEN_PARAM_INT, &roi_h, 0, NULL},
        { bumblebee_string, (char*) "tlight_focal_dist", CARMEN_PARAM_DOUBLE, &focal_distance, 0, NULL},
        { bumblebee_string, (char*) "tlight_dist_correction", CARMEN_PARAM_DOUBLE, &dist_correction, 0, NULL},
		{(char *) "traffic_light", (char *) "use_squeezenet", CARMEN_PARAM_ONOFF, &use_squeezenet, 0, NULL},
		{(char *) "traffic_light", (char *) "gpu_mode", CARMEN_PARAM_ONOFF, &gpu_mode, 0, NULL},
		{(char *) "traffic_light", (char *) "gpu_device_id", CARMEN_PARAM_INT, &gpu_device_id, 0, NULL}

    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

	carmen_param_t param_list2[] =
	{
		{(char *) "commandline",	 (char *) "generate_database",	CARMEN_PARAM_ONOFF, 	&generate_database,	 1, NULL},
		// Curiosamente eh necessario passar o endereco do (char*). Sem isso a leitura do parametro nao funciona. O carmen
		// deve alocar espaco para a string internamente.
		{(char *) "commandline",	 (char *) "database_path",		CARMEN_PARAM_STRING, 	&database_path,	 1, NULL},
	};
	carmen_param_allow_unfound_variables(true);
	carmen_param_install_params(argc, argv, param_list2, sizeof(param_list2) / sizeof(param_list2[0]));

	printf("command_line params: %d %s\n", generate_database, database_path);
    return (0);
}


void
subscribe_to_relevant_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler,
    		CARMEN_SUBSCRIBE_LATEST);
    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler,
    		CARMEN_SUBSCRIBE_LATEST);
    carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) carmen_rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    if ((argc < 2))
        carmen_die("%s: Wrong number of parameters. Traffic Light requires 1 parameters and received %d parameter(s). \n"
        		   "Usage:\n %s <camera_number>\n", argv[0], argc - 1, argv[0]);

    camera = atoi(argv[1]);

    read_parameters(argc, argv);
    carmen_traffic_light_define_messages(camera);

    signal(SIGINT, shutdown_traffic_light);
    subscribe_to_relevant_messages();

    traffic_light_module_initialization();
    carmen_ipc_dispatch();
    return (EXIT_SUCCESS);
}
