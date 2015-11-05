//CARMEN INCLUDES
#include <carmen/carmen.h>
#include <carmen/ipc_wrapper.h>
#include <carmen/visual_search_thin_interface.h>
#include <carmen/vergence_interface.h>

#include <carmen/stereo_util.h>

//ROS INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>

//ROS MSGS INCLUDES
#include "include_ros_visual_search_thin/train.h"
#include "include_ros_visual_search_thin/test.h"
#include "include_ros_visual_search_thin/output.h"

#include "include_ros_vergence/vergence_train_test.h"
#include "include_ros_vergence/vergence_output.h"

//Visual Search
ros::Subscriber visual_search_train;
ros::Subscriber visual_search_test;
ros::Publisher visual_search_saccade;

ros::Subscriber vergence_train_test;
//ros::Publisher vergence_saccade;

visual_search_thin::output ros_output_message;
visual_search_thin::output ros_vergence_output_message;

carmen_visual_search_thin_train_message carmen_visual_search_train_message;
carmen_visual_search_thin_test_message carmen_visual_search_test_message;
carmen_visual_search_thin_output_message carmen_visual_search_output_message;

carmen_vergence_train_message c_vergence_train_message;
carmen_vergence_test_message c_vergence_test_message;

carmen_vergence_train_output_message c_vergence_train_output;
carmen_vergence_test_output_message c_vergence_output_message;

//carmen_position_t search_point;

static stereo_util stereo_params;

ros::Publisher vergence_3D;
geometry_msgs::Point saccade_3D;

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		ros::shutdown();

		printf("visual_search_thin_communication: disconnected.\n");
		exit(0);
	}
}


void
copy_raw_image(unsigned char *raw_image_copy, unsigned char *raw_image, int bumblebee_basic_width, int bumblebee_basic_height)
{
	int x, y, r, g, b;

	for (y = 0; y < bumblebee_basic_height; y++)
	{
		for (x = 0; x < bumblebee_basic_width; x++)
		{
			r = raw_image[sizeof(unsigned int) * (y * bumblebee_basic_width + x) + 0];//para andar na msg do ros tem q ser assim
			g = raw_image[sizeof(unsigned int) * (y * bumblebee_basic_width + x) + 1];
			b = raw_image[sizeof(unsigned int) * (y * bumblebee_basic_width + x) + 2];

			raw_image_copy[3 * (y * bumblebee_basic_width + x) + 0] = r;//e na msg do carmen assim
			raw_image_copy[3 * (y * bumblebee_basic_width + x) + 1] = g;
			raw_image_copy[3 * (y * bumblebee_basic_width + x) + 2] = b;
		}
	}
}

void
copy_points_ros_to_carmen(carmen_position_t* carmen_point, geometry_msgs::Point* ros_point, int size){

	for(int i = 0; i < size; i++){

		carmen_point[i].x = ros_point[i].x;
		carmen_point[i].y = ros_point[i].y;
	}
}

void
copy_points_carmen_to_ros(geometry_msgs::Point* ros_point, carmen_position_t* carmen_point, int size){

	for(int i = 0; i < size; i++){

		ros_point[i].x = carmen_point[i].x;
		ros_point[i].y = carmen_point[i].y;
	}
}


void
visual_search_thin_convert_output(carmen_visual_search_thin_output_message *msg){

	ros_output_message.measured_confidence = msg->measured_confidence;

	ros_output_message.measured_scale_factor = msg->measured_scale_factor;

	ros_output_message.saccade_point.x = msg->saccade_point.x;
	ros_output_message.saccade_point.y = msg->saccade_point.y;

	ros_output_message.saccade_vector.x = msg->saccade_vector.x;
	ros_output_message.saccade_vector.y = msg->saccade_vector.y;

	visual_search_saccade.publish(ros_output_message);
}

//void
//vergence_convert_output(carmen_vergence_test_output_message *msg){
//
//	ros_vergence_output_message.measured_confidence = msg->confidence;
//
//	ros_vergence_output_message.measured_scale_factor = 1;
//
//	ros_vergence_output_message.saccade_point.x = msg->vergence_point.x;
//	ros_vergence_output_message.saccade_point.y = msg->vergence_point.y;
//
//	ros_vergence_output_message.saccade_vector.x = 0;
//	ros_vergence_output_message.saccade_vector.y = 0;
//
//	vergence_saccade.publish(ros_vergence_output_message);
//}

void
visual_search_thin_convert_train(visual_search_thin::train::Ptr ros_train_message){

	carmen_visual_search_train_message.host = carmen_get_host();

	carmen_visual_search_train_message.reference_image_size = ros_train_message->reference_image_size;

	carmen_visual_search_train_message.reference_points_size = ros_train_message->reference_points_size;

	carmen_visual_search_train_message.reference_image = (unsigned char*)malloc(carmen_visual_search_train_message.reference_image_size);

	copy_raw_image(carmen_visual_search_train_message.reference_image,
					ros_train_message->reference_image.data.data(),
					ros_train_message->reference_image.width,
					ros_train_message->reference_image.height
					);

	carmen_visual_search_train_message.reference_points = (carmen_position_t*)malloc(sizeof(carmen_position_t)*carmen_visual_search_train_message.reference_points_size);

	copy_points_ros_to_carmen(carmen_visual_search_train_message.reference_points,
				ros_train_message->reference_points.data(),
				carmen_visual_search_train_message.reference_points_size);

	carmen_visual_search_train_message.timestamp = carmen_get_time();

	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_NAME,
											&carmen_visual_search_train_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_NAME);

	free(carmen_visual_search_train_message.reference_image);
	free(carmen_visual_search_train_message.reference_points);
}

//void
//vergence_convert_train(visual_search_thin::train::Ptr ros_train_message){
//
//	c_vergence_train_message.host = carmen_get_host();
//
//	c_vergence_train_message.reference_image_size = ros_train_message->reference_image_size;
//
//	c_vergence_train_message.reference_points_size = ros_train_message->reference_points_size;
//
//	c_vergence_train_message.reference_image = (unsigned char*)malloc(c_vergence_train_message.reference_image_size);
//
//	copy_raw_image(c_vergence_train_message.reference_image,
//					ros_train_message->reference_image.data.data(),
//					ros_train_message->reference_image.width,
//					ros_train_message->reference_image.height
//					);
//
//	c_vergence_train_message.reference_points = (carmen_position_t*)malloc(sizeof(carmen_position_t)*c_vergence_train_message.reference_points_size);
//
//	copy_points_ros_to_carmen(c_vergence_train_message.reference_points,
//				ros_train_message->reference_points.data(),
//				c_vergence_train_message.reference_points_size);
//
//	c_vergence_train_message.timestamp = carmen_get_time();
//
////	printf("ENTREI NO TREINO DA VERGENCIA\n");
//	carmen_vergence_train_output_message *msg;
//	msg = carmen_vergence_query_train_message(&c_vergence_train_message, 1);
////	printf("SAI do TREIO DA VERGENCIA\n");
//
//	if(msg != NULL){
//
//		c_vergence_train_output = *msg;
//	}
//	free(c_vergence_train_message.reference_image);
//	free(c_vergence_train_message.reference_points);
//}

void
visual_search_thin_convert_test(visual_search_thin::test::Ptr ros_test_message){

	carmen_visual_search_test_message.host = carmen_get_host();

	carmen_visual_search_test_message.associated_image_size = ros_test_message->associated_image_size;

	carmen_visual_search_test_message.associated_points_size = ros_test_message->associated_points_size;

	carmen_visual_search_test_message.associated_image = (unsigned char*)malloc(carmen_visual_search_test_message.associated_image_size);

	copy_raw_image(carmen_visual_search_test_message.associated_image,
					ros_test_message->associated_image.data.data(),
					ros_test_message->associated_image.width,
					ros_test_message->associated_image.height
					);

	carmen_visual_search_test_message.associated_points = (carmen_position_t*)malloc(sizeof(carmen_position_t)*carmen_visual_search_test_message.associated_points_size);

	copy_points_ros_to_carmen(carmen_visual_search_test_message.associated_points,
				ros_test_message->associated_points.data(),
				carmen_visual_search_test_message.associated_points_size);

	carmen_visual_search_test_message.scale = ros_test_message->scale;

	carmen_visual_search_test_message.timestamp = carmen_get_time();

	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_NAME,
											&carmen_visual_search_test_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_NAME);

	free(carmen_visual_search_test_message.associated_image);
	free(carmen_visual_search_test_message.associated_points);
}

//void
//vergence_convert_test(visual_search_thin::test::Ptr ros_test_message){
//
//	c_vergence_test_message.host = carmen_get_host();
//
//	c_vergence_test_message.associated_image_size = ros_test_message->associated_image_size;
//
//	c_vergence_test_message.associated_points_size = ros_test_message->associated_points_size;
//
//	c_vergence_test_message.associated_image = (unsigned char*)malloc(c_vergence_test_message.associated_image_size);
//
//	copy_raw_image(c_vergence_test_message.associated_image,
//					ros_test_message->associated_image.data.data(),
//					ros_test_message->associated_image.width,
//					ros_test_message->associated_image.height
//					);
//
//	c_vergence_test_message.associated_points = (carmen_position_t*)malloc(sizeof(carmen_position_t)*c_vergence_test_message.associated_points_size);
//
//	copy_points_ros_to_carmen(c_vergence_test_message.associated_points,
//				ros_test_message->associated_points.data(),
//				c_vergence_test_message.associated_points_size);
//
//	c_vergence_test_message.timestamp = carmen_get_time();
//
////	printf("ENTREI NO TESTE DA VERGENCIA\n");
//	carmen_vergence_test_output_message *msg;
//	msg = carmen_vergence_query_test_message(&c_vergence_test_message, 1);
////	printf("SAI do TESTE DA VERGENCIA\n");
//
//	if(msg != NULL){
//
//		c_vergence_output_message = *msg;
//		vergence_convert_output(msg);
////		printf("OUTPUT: %f %f \n",
////						c_vergence_output_message.vergence_point.x,
////						c_vergence_output_message.vergence_point.y
////				);
//
//	}
//
//	free(c_vergence_test_message.associated_image);
//	free(c_vergence_test_message.associated_points);
//}

void
train_handler(visual_search_thin::train::Ptr ros_train_message){

	visual_search_thin_convert_train(ros_train_message);
//	vergence_convert_train(ros_train_message);
}

void
test_handler(visual_search_thin::test::Ptr ros_test_message){

	visual_search_thin_convert_test(ros_test_message);
//	vergence_convert_test(ros_test_message);
}

void
output_message_handler(carmen_visual_search_thin_output_message* msg){

	carmen_visual_search_output_message = *msg;
	//preenche a mensagem do ros e publica
	visual_search_thin_convert_output(&carmen_visual_search_output_message);
}

int
train_vergence(sensor_msgs::Image image, int points_size, geometry_msgs::Point* point_right, int image_size){

	c_vergence_train_message.host = carmen_get_host();

	c_vergence_train_message.reference_image_size = image_size;

	c_vergence_train_message.reference_points_size = points_size;

	c_vergence_train_message.reference_image = (unsigned char*)malloc(c_vergence_train_message.reference_image_size);

	copy_raw_image(c_vergence_train_message.reference_image,
			image.data.data(),
			image.width,
			image.height
	);

	c_vergence_train_message.reference_points = (carmen_position_t*)malloc(sizeof(carmen_position_t)*c_vergence_train_message.reference_points_size);

	copy_points_ros_to_carmen(c_vergence_train_message.reference_points,
			point_right,
			c_vergence_train_message.reference_points_size);

	c_vergence_train_message.timestamp = carmen_get_time();

	carmen_vergence_train_output_message *msg;
	msg = carmen_vergence_query_train_message(&c_vergence_train_message, 20.0);

	int retorno = 0;

	if(msg != NULL){

		c_vergence_train_output = *msg;
		retorno = 1;
	}else{
		retorno = 0;
	}

	free(c_vergence_train_message.reference_image);
	free(c_vergence_train_message.reference_points);

	return retorno;
}

int
test_vergence(sensor_msgs::Image image, int points_size, geometry_msgs::Point* point_left, int image_size){

	c_vergence_test_message.host = carmen_get_host();

	c_vergence_test_message.associated_image_size = image_size;

	c_vergence_test_message.associated_points_size = points_size;

	c_vergence_test_message.associated_image = (unsigned char*)malloc(c_vergence_test_message.associated_image_size);

	copy_raw_image(c_vergence_test_message.associated_image,
			image.data.data(),
			image.width,
			image.height
	);

	c_vergence_test_message.associated_points = (carmen_position_t*)malloc(sizeof(carmen_position_t)*c_vergence_test_message.associated_points_size);

	copy_points_ros_to_carmen(c_vergence_test_message.associated_points,
			point_left,
			c_vergence_test_message.associated_points_size);

	c_vergence_test_message.timestamp = carmen_get_time();

	carmen_vergence_test_output_message *msg;
	msg = carmen_vergence_query_test_message(&c_vergence_test_message, 20.0);

	int retorno = 0;
	if(msg != NULL){

		c_vergence_output_message = *msg;
//		vergence_convert_output(msg);
		retorno = 1;
	}else{
		retorno = 0;
	}

	free(c_vergence_test_message.associated_image);
	free(c_vergence_test_message.associated_points);

	return retorno;
}

void
handler_vergence(cyton_kinect::vergence_train_test::Ptr ros_vergence_messsage){

	int disparity = -1;
	carmen_position_t point_right;

	carmen_vector_3D_p p3d = (carmen_vector_3D_p)malloc(sizeof(carmen_vector_3D_t));

	if(train_vergence(ros_vergence_messsage->reference_image, ros_vergence_messsage->reference_points_size,
						ros_vergence_messsage->reference_points.data(), ros_vergence_messsage->reference_image_size)){

//			point_right.x = ros_vergence_messsage->reference_points.data()[0].x;
//			point_right.y = ros_vergence_messsage->reference_points.data()[0].y;
		//convert point to 640x480(original) format
		point_right.x = ros_vergence_messsage->reference_points.data()[0].x*2;
//		point_right.y = ((ros_vergence_messsage->reference_image.height-1) - ros_vergence_messsage->reference_points.data()[0].y)*2;
		point_right.y = ros_vergence_messsage->reference_points.data()[0].y*2;

		if(test_vergence(ros_vergence_messsage->associated_image, ros_vergence_messsage->associated_points_size,
						ros_vergence_messsage->associated_points.data(), ros_vergence_messsage->associated_image_size)){

			//is needed to mutiply x per 2 to convert to original format(640)
			disparity = (int)(c_vergence_output_message.vergence_point.x*2 - point_right.x);
			ROS_WARN("%d",disparity);
			p3d = reproject_single_point_to_3D(&stereo_params, point_right, disparity);
			if(p3d != NULL){

				saccade_3D.x = p3d->x-1.4;
				saccade_3D.y = p3d->y;
				saccade_3D.z = p3d->z+0.26;
				vergence_3D.publish(saccade_3D);
//				printf("X %f Y %f Z %f\n", p3d->x, p3d->y, p3d->z);
			}
		}
	}
	free(p3d);
}

int
main(int argc, char **argv)
{
	IPC_initialize();
	IPC_connect(argv[0]);

	signal(SIGINT, shutdown_module);

	carmen_visual_search_thin_define_message_train();
	carmen_visual_search_thin_define_message_test();
	carmen_visual_search_thin_define_message_output();

	carmen_vergence_define_message_output_train();
	carmen_vergence_define_message_output_test();

	carmen_vergence_define_message_query_train();
	carmen_vergence_define_message_query_test();

	stereo_params = get_stereo_instance(6, 0, 0);//pega os parametros da camera

	ros::init(argc, argv, "visual_search_communication", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	visual_search_train = nh.subscribe("/visual_search/train", 1, train_handler);
	visual_search_test = nh.subscribe("/visual_search/test", 1, test_handler);

	vergence_train_test = nh.subscribe("/vergence/train_test", 1, handler_vergence);

	visual_search_saccade = nh.advertise<visual_search_thin::output>("/visual_search/saccade", 1);
//	vergence_saccade = nh.advertise<visual_search_thin::output>("/vergence/saccade", 1);
	vergence_3D = nh.advertise<geometry_msgs::Point>("/vergence/saccade_3D", 1);

	carmen_visual_search_thin_subscribe_output(NULL, (carmen_handler_t) output_message_handler, CARMEN_SUBSCRIBE_LATEST);

	ros::Rate loop_rate(100);
	while (1)
	{
		carmen_ipc_sleep(0.01);
        ros::spinOnce();
        loop_rate.sleep();
	}
	return 0;
}
