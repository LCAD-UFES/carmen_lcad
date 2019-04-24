#include <carmen/carmen.h>
#include <carmen/laser_ldmrs_messages.h>


void
carmen_laser_define_ldmrs_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LASER_LDMRS_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LASER_LDMRS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LDMRS_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LDMRS_OBJECTS_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_LASER_LDMRS_OBJECTS_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LDMRS_OBJECTS_NAME);

	err = IPC_defineMsg(CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME, IPC_VARIABLE_LENGTH,
					CARMEN_LASER_LDMRS_OBJECTS_DATA_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME);
}


void
carmen_laser_define_ldmrs_new_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LASER_LDMRS_NEW_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LASER_LDMRS_NEW_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LDMRS_NEW_NAME);
}


void
carmen_laser_subscribe_ldmrs_message(carmen_laser_ldmrs_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*)CARMEN_LASER_LDMRS_NAME,
			(char*)CARMEN_LASER_LDMRS_FMT,
			message, sizeof(carmen_laser_ldmrs_message),
			handler, subscribe_how);
}


void
carmen_laser_subscribe_ldmrs_new_message(carmen_laser_ldmrs_new_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*)CARMEN_LASER_LDMRS_NEW_NAME,
			(char*)CARMEN_LASER_LDMRS_NEW_FMT,
			message, sizeof(carmen_laser_ldmrs_new_message),
			handler, subscribe_how);
}


void
carmen_laser_unsubscribe_ldmrs(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char*)CARMEN_LASER_LDMRS_NAME, handler);
}


void
carmen_laser_unsubscribe_ldmrs_new(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char*)CARMEN_LASER_LDMRS_NEW_NAME, handler);
}


IPC_RETURN_TYPE
carmen_laser_publish_ldmrs(carmen_laser_ldmrs_message *message)
{
	IPC_RETURN_TYPE err;

	message->host = carmen_get_host();

	err = IPC_publishData(CARMEN_LASER_LDMRS_NAME, (carmen_laser_ldmrs_message *)message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LASER_LDMRS_NAME);

	return err;
}


IPC_RETURN_TYPE
carmen_laser_publish_ldmrs_new(carmen_laser_ldmrs_new_message *message)
{
	IPC_RETURN_TYPE err;

	message->host = carmen_get_host();

	err = IPC_publishData(CARMEN_LASER_LDMRS_NEW_NAME, (carmen_laser_ldmrs_new_message *)message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LASER_LDMRS_NEW_NAME);

	return err;
}


//ldmrs objects
void
carmen_laser_subscribe_ldmrs_objects_message(carmen_laser_ldmrs_objects_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*)CARMEN_LASER_LDMRS_OBJECTS_NAME,
				(char*)CARMEN_LASER_LDMRS_OBJECTS_FMT,
				message, sizeof(carmen_laser_ldmrs_objects_message),
				handler, subscribe_how);
}

void
carmen_laser_unsubscribe_ldmrs_objects(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char*)CARMEN_LASER_LDMRS_OBJECTS_NAME, handler);
}

IPC_RETURN_TYPE
carmen_laser_publish_ldmrs_objects(carmen_laser_ldmrs_objects_message *message)
{
	IPC_RETURN_TYPE err;

	message->host = carmen_get_host();
	message->timestamp = carmen_get_time();

	err = IPC_publishData(CARMEN_LASER_LDMRS_OBJECTS_NAME, (carmen_laser_ldmrs_objects_message *)message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LASER_LDMRS_OBJECTS_NAME);

	return err;
}


//ldmrs objects data
void
carmen_laser_subscribe_ldmrs_objects_data_message(carmen_laser_ldmrs_objects_data_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*)CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME,
				(char*)CARMEN_LASER_LDMRS_OBJECTS_DATA_FMT,
				message, sizeof(carmen_laser_ldmrs_objects_data_message),
				handler, subscribe_how);
}

void
carmen_laser_unsubscribe_ldmrs_objects_data(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char*)CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME, handler);
}

IPC_RETURN_TYPE
carmen_laser_publish_ldmrs_objects_data(carmen_laser_ldmrs_objects_data_message *message)
{
	IPC_RETURN_TYPE err;

	message->host = carmen_get_host();

	err = IPC_publishData(CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME, (carmen_laser_ldmrs_objects_data_message *)message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME);

	return err;
}
