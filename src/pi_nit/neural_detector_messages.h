/*
 * neural_detector_messages.h
 *
 *  Created on: 15/02/2022
 *      Author: Pedro
 */

#ifndef NEURAL_DETECTOR_MESSAGES_H_
#define NEURAL_DETECTOR_MESSAGES_H_

typedef struct
{
	unsigned int x, y, w, h;	// (x,y) - top-left corner, (w, h) - width & height of bounded box
	float prob;					// confidence - probability that the object was found correctly
	int obj_id;		// class of object - from range [0, classes-1]
	int track_id;		// tracking id for video (0 - untracked, 1 - inf - tracked object)
} bbox_i;

typedef struct
{
    int num_detected_objects;
    bbox_i *detected_objects;
    double timestamp;
    char *host;
} neural_detector_message;

#define		NEURAL_DETECTOR_MESSAGE_NAME		"neural_detector_message"
#define		NEURAL_DETECTOR_MESSAGE_FMT		"{int, <{int, int, int, int, float, int, int}:1>, double, string}"

#define NEURAL_DETECTOR_MESSAGE_0_NAME 	"neural_detector_message_0_name"
#define NEURAL_DETECTOR_MESSAGE_1_NAME 	"neural_detector_message_1_name"
#define NEURAL_DETECTOR_MESSAGE_2_NAME 	"neural_detector_message_2_name"
#define NEURAL_DETECTOR_MESSAGE_3_NAME 	"neural_detector_message_3_name"
#define NEURAL_DETECTOR_MESSAGE_4_NAME 	"neural_detector_message_4_name"

#endif /* NEURAL_OBJECT_TRACKER_MESSAGES_H_ */
