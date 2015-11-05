#include <carmen/carmen.h>
#include <carmen/xsensOdometer_messages.h>

void
carmen_xsensOdometer_subscribe_xsensOdometer_message(carmen_xsens_odometry_message
					     *xsens_odometry,
					     carmen_handler_t handler,
					     carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_XSENS_ODOMETRY_NAME, 
                           CARMEN_XSENS_ODOMETRY_FMT,
                           xsens_odometry, sizeof(carmen_xsens_odometry_message), 
			   handler, subscribe_how);
}

void
carmen_xsensOdometer_unsubscribe_xsensOdometer_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_XSENS_ODOMETRY_NAME, handler);
}

