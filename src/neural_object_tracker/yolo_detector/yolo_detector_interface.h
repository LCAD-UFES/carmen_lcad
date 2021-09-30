
#ifndef _YOLO_DETECTOR_INTERFACE_H_
#define _YOLO_DETECTOR_INTERFACE_H_


#include <carmen/carmen.h>
#include <carmen/yolo_detector_interface.h>

#ifdef __cplusplus
extern "C"
{ 
#endif

	void
	yolo_detector_subscribe_yolo_detector_message(yolo_detector_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	yolo_detector_unsubscribe_yolo_detector_message(carmen_handler_t handler);

	void
	yolo_detector_define_messages();

	void
	yolo_detector_publish_yolo_detector_message(yolo_detector_message *yolo_detector_yolo_detector_message);

#ifdef __cplusplus
}
#endif

#endif