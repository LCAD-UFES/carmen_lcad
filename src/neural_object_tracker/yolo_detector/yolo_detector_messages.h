/*
 * yolo_detector_messages.h
 *
 *  Created on: 30/09/2021
 *      Author: Pedro
 */

#ifndef YOLO_DETECTOR_MESSAGES_H_
#define YOLO_DETECTOR_MESSAGES_H_

#include <carmen/carmen.h>
#include <carmen/camera_drivers_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int x;
    int y;
    int w;
    int h;
    double prob;
    int obj_id;
    int track_id;
} bboxes;

typedef struct
{
    int cam_id;
    int qtd_bboxes;
    bboxes *bounding_boxes;
} yolo_detector_message;

#define		YOLO_DETECTOR_MESSAGE_NAME		"yolo_detector_message"
#define		YOLO_DETECTOR_MESSAGE_FMT		"{int, int, <{int, int, int, int, double, int, int}:2>}"

#ifdef __cplusplus
}
#endif

#endif /* YOLO_DETECTOR_MESSAGES_H_ */