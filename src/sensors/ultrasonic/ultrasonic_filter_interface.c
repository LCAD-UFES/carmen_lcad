#include <carmen/carmen.h>
#include <carmen/ultrasonic_filter_messages.h>

void
carmen_ultrasonic_sonar_sensor_subscribe(carmen_ultrasonic_sonar_sensor_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, 
                           CARMEN_ULTRASONIC_SONAR_SENSOR_FMT,
                           message, sizeof(carmen_ultrasonic_sonar_sensor_message), 
			   handler, subscribe_how);
}


void
carmen_ultrasonic_sonar_sensor_unsubscribe(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, handler);
}


void
carmen_ultrasonic_sonar_sensor_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, IPC_VARIABLE_LENGTH, 
	CARMEN_ULTRASONIC_SONAR_SENSOR_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_ULTRASONIC_SONAR_SENSOR_NAME); 
}

