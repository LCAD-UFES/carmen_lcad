#include <carmen/carmen.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>

#include <iostream>
#include <cstdio>
#include <vector>
#include <list>
#include <string>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/contrib/detection_based_tracker.hpp>

#include "dlib/svm.h"

#define WIDTH 9
#define HEIGHT 20
#define RED 0
#define GREEN 1

using namespace dlib;
using namespace cv;
using namespace std;

//Parameters

//SVM
typedef matrix<double, 180, 1> sample_type;
typedef radial_basis_kernel<sample_type> kernel_type;
typedef decision_function<kernel_type> dec_funct_type;
typedef normalized_function<dec_funct_type> funct_type;
funct_type learned_function;
static sample_type sample;
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
carmen_localize_ackerman_globalpos_message localize_message;
carmen_mapping_traffic_light_message mapping_traffic_light_message;
carmen_traffic_light_message traffic_light_message;
carmen_bumblebee_basic_stereoimage_message stereo_image_message;

struct semaphore
{
    int num;
    int x1;
    int x2;
    int y1;
    int y2;
    int color;
};
static int num = 0;
static std::vector<semaphore> last;
static semaphore traffic_light;

/**
 * Method to initialize the message
 */
void
traffic_light_algorithm_initialization()
{
    traffic_light_message.traffic_light_size = image_height * image_width * 3;
    traffic_light_message.signals = 0;
    traffic_light_message.distance = 0.0;
    traffic_light_message.state = NULL;
    traffic_light_message.host = carmen_get_host();
    traffic_light_message.timestamp = carmen_get_time();

    //Read the haar cascade trained
    ts_cascade.load(ts_cascade_name);

    //Read the svm trained
    ifstream fin(svm_train_name.c_str(), ios::binary);
    deserialize(learned_function, fin);
}

/**
 * Reading parameters of initialize
 * @param argc argc of terminal
 * @param argv argv of terminal
 * @return success 
 */
static int
read_parameters(int argc, char **argv)
{
    int num_items;
    char bumblebee_string[256];

    if (argc == 2)
        camera = atoi(argv[1]);
    else
    {
        printf("Usage: %s %s", argv[0], argv[1]);
        exit(0);
    }

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

    carmen_param_t param_list[] = {
        { bumblebee_string, (char*) "width", CARMEN_PARAM_INT, &image_width, 0,
            NULL},
        { bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &image_height, 0,
            NULL}
    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}

/**
 * Method for look if has a traffic light by mapping traffic light
 * @return If have or no a traffic light with a distance of 200 meters
 */
int
have_traffic_light()
{
    if (mapping_traffic_light_message.distance < 200)
    {
        traffic_light_message.distance = mapping_traffic_light_message.distance;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * Method for read image for test in svm the state of traffic light
 * @param image image of haar cascade
 */
void
read_image_svm(cv::Mat image)
{
    cv::Mat resized_image;

    resized_image.create(HEIGHT, WIDTH, CV_8UC3);

    resize(image, resized_image, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_CUBIC);

    std::vector<cv::Mat> channels;
    split(resized_image, channels);

    cv::Mat aux;
    aux = ((channels.at(0) - channels.at(1)) + 255) / 2;

    for (int x = 0; x < resized_image.rows; x++)
    {
        for (int y = 0; y < resized_image.cols; y++)
        {
            sample(x * WIDTH + y) = (double) (aux.at<uchar>(x, y));
        }
    }
}

/**
 * Method for detect and determine the state of traffic light
 * @param frame image to detect the traffic light
 * @return image with traffic light detected
 */

cv::Mat
detect(cv::Mat frame)
{
    num++;
    std::vector<Rect> semaphores;
    cv::Mat frame_gray;

    Rect ROI(Point(image_width / 4, 0), Point(image_width / 4 * 3, image_height / 2));
    cv::Mat half_image;

    cv::Mat(frame, ROI).copyTo(half_image);

    cvtColor(half_image, frame_gray, CV_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    //-- Detect traffic lights  
    ts_cascade.detectMultiScale(frame_gray, semaphores, 1.1, 3, 0, Size(0, 0), Size(200, 400));

    traffic_light_message.signals = semaphores.size();

    for (size_t i = 0; i < semaphores.size(); i++)
    {
        CvPoint p1, p2;
        p1.x = semaphores[i].x + image_width / 4;
        p1.y = semaphores[i].y;
        p2.x = p1.x + semaphores[i].width;
        p2.y = p1.y + semaphores[i].height;

        Rect myROI(p1, p2);
        cv::Mat croppedImage;

        cv::Mat(frame, myROI).copyTo(croppedImage);

        read_image_svm(croppedImage);

        if (learned_function(sample) >= 0)
        {
            circle(frame, Point(50, 130), 50, Scalar(255, 0, 0), -1, 8);
            cv::rectangle(frame, p1, p2, CV_RGB(0, 0, 255), 3, 10, 0);

            traffic_light.color = RED;
            traffic_light.num = num;
            traffic_light.x1 = p1.x;
            traffic_light.y1 = p1.y;
            traffic_light.x2 = p2.x;
            traffic_light.y2 = p2.y;
            last.push_back(traffic_light);
            traffic_light_message.state = (char *) "R";
    ofstream out;
    out.open("saida.txt", std::fstream::out | std::fstream::app);
    out << traffic_light_message.distance << " "<<num<<" 1 0"<<endl;
    out.close();
        }
        else if (learned_function(sample) < 0)
        {
            circle(frame, Point(50, 130), 50, Scalar(0, 255, 0), -1, 8);
            cv::rectangle(frame, p1, p2, CV_RGB(0, 255, 0), 3, 10, 0);

            traffic_light.color = GREEN;
            traffic_light.num = num;
            traffic_light.x1 = p1.x;
            traffic_light.y1 = p1.y;
            traffic_light.x2 = p2.x;
            traffic_light.y2 = p2.y;
            last.push_back(traffic_light);
            traffic_light_message.state = (char *) "G";
ofstream out;
    out.open("saida.txt", std::fstream::out | std::fstream::app);
    out << traffic_light_message.distance << " "<<num<<" 0 1"<<endl;
    out.close();
}
    }

    ostringstream myStream; //creates an ostringstream object
    myStream << traffic_light_message.distance << flush;
    string text;
    if (traffic_light_message.distance <=200 && traffic_light_message.distance != -1)
    {
        text = "Distance " + myStream.str()+ " meters";
    }
    else
        text = "Distance greater than 200 meters";

    int fontFace = FONT_HERSHEY_COMPLEX;
    double fontScale = 2;
    int thickness = 3;

    putText(frame, text, Point(20, 900), fontFace, fontScale, Scalar(255, 255, 255), thickness, 8);
      
    return frame;
}

/**
 * Method for compute the state of traffic light
 * @param stereo_image
 */
void
compute_traffic_light(carmen_bumblebee_basic_stereoimage_message * stereo_image)
{
    stereo_image_message = *stereo_image;

    cv::Mat image;
    image.create(image_height, image_width, CV_8UC3);
    image.data = (uchar *) stereo_image->raw_right;

    //Detecting
    detect(image);

    //coping the image to message
    traffic_light_message.traffic_light_image = (uchar*) image.data;
}

/**
 * Method of handler of mapping_traffic_light
 * @param carmen_mapping_traffic_light_message Message of mapping traffic light
 */
void
mapping_traffic_light_handler(carmen_mapping_traffic_light_message * msg)
{

    mapping_traffic_light_message = *msg;
}

/**
 * Main method for mapping, detecting and inferring the traffic light state
 * @param stereo_image Message of stereo camera
 */
void
traffic_light_handler(carmen_bumblebee_basic_stereoimage_message * stereo_image)
{
    if (have_traffic_light()) //computing that has traffic light
    {
        traffic_light_message.distance = mapping_traffic_light_message.distance;
        compute_traffic_light(stereo_image);
        traffic_light_message.timestamp = carmen_get_time();

    }
    else //putting the message that has no traffic lights
    {

        traffic_light_message.timestamp = carmen_get_time();
        traffic_light_message.traffic_light_image = stereo_image->raw_left;
        traffic_light_message.distance = infinite;
        traffic_light_message.signals = 0;
        traffic_light_message.state = NULL;
    }
    //publish the message
    carmen_traffic_light_publish_message(camera, &traffic_light_message);
}

void
localize_ackerman_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    localize_message = *msg;
}

/**
 * Method for subscribe messages of localize and mapping of traffic light
 */
void
subscribe_camera_mapping_traffic_light_messages()
{
    //carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_mapping_traffic_light_subscribe(NULL, (carmen_handler_t) mapping_traffic_light_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) traffic_light_handler, CARMEN_SUBSCRIBE_LATEST);
}

/**
 * Main of program
 * @param argc
 * @param argv
 * @return 
 */
int
main(int argc, char **argv)
{
    /* connect to IPC server */

    carmen_ipc_initialize(argc, argv);

    carmen_param_check_version(argv[0]);

    if ((argc != 2))
        carmen_die(
                   "%s: Wrong number of parameters. Traffic Light requires 1 parameters and received %d parameter(s). \nUsage:\n %s <camera_number>\n",
                   argv[0], argc - 1, argv[0]);

    camera = atoi(argv[1]);

    read_parameters(argc, argv);

    carmen_traffic_light_define_messages(camera);

    /* Subscribe messages of camera and mapping of traffic light*/
    subscribe_camera_mapping_traffic_light_messages();

    traffic_light_algorithm_initialization();

    /* Loop forever waiting for messages */
    carmen_ipc_dispatch();

    return EXIT_SUCCESS;
}
