#include <carmen/laser_ldmrs_messages.h>

#ifdef __cplusplus
extern "C" 
{
#endif

void
carmen_laser_define_ldmrs_messages();

//ldmrs point cloud
void
carmen_laser_subscribe_ldmrs_message(carmen_laser_ldmrs_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_laser_unsubscribe_ldmrs(carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_laser_publish_ldmrs(carmen_laser_ldmrs_message *message);


//ldmrs objects
void
carmen_laser_subscribe_ldmrs_objects_message(carmen_laser_ldmrs_objects_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_laser_unsubscribe_ldmrs_objects(carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_laser_publish_ldmrs_objects(carmen_laser_ldmrs_objects_message *message);

//ldmrs objects data
void
carmen_laser_subscribe_ldmrs_objects_data_message(carmen_laser_ldmrs_objects_data_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_laser_unsubscribe_ldmrs_objects_data(carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_laser_publish_ldmrs_objects_data(carmen_laser_ldmrs_objects_data_message *message);

#ifdef __cplusplus
}
#endif


// @}

