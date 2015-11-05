#include "ipc.h"
#include "messages/ros_bumblebee_basic_messages.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


using namespace sensor_msgs;
using namespace message_filters;


int image_capture_frequency;
double time_last_image_has_been_captured;
carmen_bumblebee_basic_stereoimage_message carmen_stereo_image_pair_message;
carmen_bumblebee_basic_stereoimage_message carmen_stereo_hand_image_pair_message;


void
calculate_image_capture_frequency()
{
  double time_now = time(NULL);

  if ((time_now - time_last_image_has_been_captured) > 1.0)
  {

    printf("num images per second: %d\n", image_capture_frequency);
    time_last_image_has_been_captured = time_now;
    image_capture_frequency = 0;
  }
  else
    image_capture_frequency++;
}


void
generate_bumblebee_image_message(const sensor_msgs::Image::ConstPtr& left_image_message, const sensor_msgs::Image::ConstPtr& right_image_message, carmen_bumblebee_basic_stereoimage_message* carmen_stereo_image_pair_message_ptr)
{
  int image_size = (left_image_message->width * left_image_message->height * 3);

  carmen_stereo_image_pair_message_ptr->width = left_image_message->width;
  carmen_stereo_image_pair_message_ptr->height = left_image_message->height;
  carmen_stereo_image_pair_message_ptr->image_size = image_size;
  carmen_stereo_image_pair_message_ptr->isRectified = 1;

  if (carmen_stereo_image_pair_message_ptr->raw_left == 0)
    carmen_stereo_image_pair_message_ptr->raw_left = (unsigned char*) calloc (image_size, sizeof(unsigned char));

  if (carmen_stereo_image_pair_message_ptr->raw_right == 0)
    carmen_stereo_image_pair_message_ptr->raw_right = (unsigned char*) calloc (image_size, sizeof(unsigned char));

  if (left_image_message->step == left_image_message->width)
  {
  	for (unsigned int i = 0; i < left_image_message->height; i++)
  	{
	    for (unsigned int j = 0; j < left_image_message->width; j++)
    		{
      
		        carmen_stereo_image_pair_message_ptr->raw_left[3 * i * left_image_message->width + 3 * j + 0] = left_image_message->data[i * left_image_message->width + j];
		        carmen_stereo_image_pair_message_ptr->raw_left[3 * i * left_image_message->width + 3 * j + 1] = left_image_message->data[i * left_image_message->width + j];
			carmen_stereo_image_pair_message_ptr->raw_left[3 * i * left_image_message->width + 3 * j + 2] = left_image_message->data[i * left_image_message->width + j];
	
		        carmen_stereo_image_pair_message_ptr->raw_right[3 * i * right_image_message->width + 3 * j + 0] = right_image_message->data[i * right_image_message->width + j];
		        carmen_stereo_image_pair_message_ptr->raw_right[3 * i * right_image_message->width + 3 * j + 1] = right_image_message->data[i * right_image_message->width + j];
			carmen_stereo_image_pair_message_ptr->raw_right[3 * i * right_image_message->width + 3 * j + 2] = right_image_message->data[i * right_image_message->width + j];
      		}
    	}
  }
  else
  {
  	memcpy(carmen_stereo_image_pair_message_ptr->raw_left, (unsigned char*) &(left_image_message->data[0]), image_size);
	memcpy(carmen_stereo_image_pair_message_ptr->raw_right, (unsigned char*) &(right_image_message->data[0]), image_size);
  }

  carmen_stereo_image_pair_message_ptr->timestamp = ((double) left_image_message->header.stamp.nsec + right_image_message->header.stamp.nsec) / 2;
}


void
publish_bumblebee_image_message(carmen_bumblebee_basic_stereoimage_message carmen_stereo_image_pair_message_param)
{
  IPC_publishData(CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME, &carmen_stereo_image_pair_message_param);
}

void
publish_bumblebee_hand_image_message(carmen_bumblebee_basic_stereoimage_message carmen_stereo_hand_image_pair_message_param)
{
  IPC_publishData(CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE11_NAME, &carmen_stereo_hand_image_pair_message_param);
}

void
image_callback(const sensor_msgs::Image::ConstPtr& left_image_message, const sensor_msgs::Image::ConstPtr& right_image_message)
{
  ROS_INFO("Left Image: %dx%d (%d) Right Image: %dx%d (%d)", 
    left_image_message->width, left_image_message->height, left_image_message->header.stamp.nsec,
    right_image_message->width, right_image_message->height, right_image_message->header.stamp.nsec);

  generate_bumblebee_image_message(left_image_message, right_image_message, &carmen_stereo_image_pair_message);
  publish_bumblebee_image_message(carmen_stereo_image_pair_message);
}


void
hand_image_callback(const sensor_msgs::Image::ConstPtr& left_image_message, const sensor_msgs::Image::ConstPtr& right_image_message)
{
  ROS_INFO("Left Image: %dx%d (%d) Right Image: %dx%d (%d)", 
    left_image_message->width, left_image_message->height, left_image_message->header.stamp.nsec,
    right_image_message->width, right_image_message->height, right_image_message->header.stamp.nsec);

  generate_bumblebee_image_message(left_image_message, right_image_message, &carmen_stereo_hand_image_pair_message);
  publish_bumblebee_hand_image_message(carmen_stereo_hand_image_pair_message);
}

void
initialize_global_data()
{
  memset(&carmen_stereo_image_pair_message, 0, sizeof(carmen_stereo_image_pair_message));
  memset(&carmen_stereo_hand_image_pair_message, 0, sizeof(carmen_stereo_hand_image_pair_message));
  
  carmen_stereo_image_pair_message.host = (char*) malloc (64 * sizeof(char)); 
  strcpy(carmen_stereo_image_pair_message.host, "localhost");

  carmen_stereo_hand_image_pair_message.host = (char*) malloc (64 * sizeof(char)); 
  strcpy(carmen_stereo_hand_image_pair_message.host, "localhost");

  image_capture_frequency = 0;
  time_last_image_has_been_captured = 0;
}


int
main(int argc, char **argv)
{
  IPC_initialize();
  IPC_connect(argv[0]);

  IPC_defineMsg(CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME, IPC_VARIABLE_LENGTH,  CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_FMT);
  IPC_defineMsg(CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE11_NAME, IPC_VARIABLE_LENGTH,  CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE11_FMT);
  
  initialize_global_data();

  ros::init(argc, argv, "carmen_ros_communication");
  ros::NodeHandle n;

  message_filters::Subscriber<sensor_msgs::Image> left_image_subscriber (n, "/multisense_sl/camera/left/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> right_image_subscriber (n, "/multisense_sl/camera/right/image_rect_color", 1);

  message_filters::Subscriber<sensor_msgs::Image> left_hand_image_subscriber (n, "/sandia_hands/l_hand/camera/left/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> right_hand_image_subscriber (n, "/sandia_hands/l_hand/camera/right/image_rect_color", 1);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy; // match messages with close timestamps <- faster
  typedef sync_policies::ApproximateTime<Image, Image> MyHandImageSyncPolicy; // match messages with close timestamps <- faster
  //typedef sync_policies::ExactTime<Image, Image> MySyncPolicy; // match messages with the same timestamp <- slower and unbounded on time

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_image_subscriber, right_image_subscriber);
  Synchronizer<MyHandImageSyncPolicy> hand_sync(MyHandImageSyncPolicy(10), left_hand_image_subscriber, right_hand_image_subscriber);
  
  sync.registerCallback(boost::bind(&image_callback, _1, _2));
  hand_sync.registerCallback(boost::bind(&hand_image_callback, _1, _2));

  ros::spin();

  IPC_disconnect();
  return 0;
}


