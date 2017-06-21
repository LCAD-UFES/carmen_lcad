/*
 * message.h
 *
 *  Created on: Jun 13, 2017
 *      Author: avelino
 */

#ifndef CARMEN_LOCALIZE_NEURAL_MESSAGES_H_
#define CARMEN_LOCALIZE_NEURAL_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	int 				height;
	int 				width;
	int 				size;
	char 				*image_data;
	carmen_pose_3D_t 	pose;
	double 				timestamp;
	char 				*host;
} carmen_localize_neural_imagepos_message;

#define CARMEN_LOCALIZE_NEURAL_IMAGEPOS_KEYFRAME_NAME "carmen_localize_neural_imagepos_keyframe_message"
#define CARMEN_LOCALIZE_NEURAL_IMAGEPOS_CURFRAME_NAME "carmen_localize_neural_imagepos_curframe_message"
#define CARMEN_LOCALIZE_NEURAL_IMAGEPOS_FMT "{int, int, int, <{char}:3>, {{double,double,double},{double,double,double}}, double, string}"

#ifdef __cplusplus
}
#endif

#endif /* CARMEN_LOCALIZE_NEURAL_MESSAGES_H_ */
