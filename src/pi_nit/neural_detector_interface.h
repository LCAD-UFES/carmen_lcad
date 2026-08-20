
#ifndef _NEURAL_DETECTOR_INTERFACE_H_
#define _NEURAL_DETECTOR_INTERFACE_H_


#include <carmen/carmen.h>
#include <carmen/neural_detector_messages.h>

#ifdef __cplusplus
extern "C"
{ 
#endif

	void
	neural_detector_subscribe_message(int message_id, neural_detector_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	neural_detector_unsubscribe_message(int message_id, carmen_handler_t handler);

	IPC_RETURN_TYPE
	neural_detector_define_message(int message_id);

	IPC_RETURN_TYPE
	neural_detector_publish_message(int message_id, neural_detector_message *neural_detector_neural_detector_message);

#ifdef __cplusplus
}
#endif

#endif