#include <carmen/carmen.h>
#include "zed_camera_sensor_messages.h"

void
carmen_zed_camera_sensor_subscribe_stereoimage(carmen_zed_camera_sensor_stereoimage_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_NAME,
                             CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_FMT,
                             message, sizeof(carmen_zed_camera_sensor_stereoimage_message),
                             handler, subscribe_how);
}

void
carmen_zed_camera_sensor_subscribe_depthmap(carmen_zed_camera_sensor_depthmap_message *message,
                                               carmen_handler_t handler,
                                               carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_NAME,
                             CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_FMT,
                             message, sizeof(carmen_zed_camera_sensor_depthmap_message),
                             handler, subscribe_how);
}

void
carmen_zed_camera_sensor_unsubscribe_stereoimage(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_NAME, handler);
}

void
carmen_zed_camera_sensor_unsubscribe_depthmap(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_NAME, handler);
}


IPC_RETURN_TYPE
carmen_zed_camera_sensor_define_stereoimage_messages()
{
    IPC_RETURN_TYPE err;

    err = IPC_defineMsg(CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_NAME);
    return err;
}

IPC_RETURN_TYPE
carmen_zed_camera_sensor_define_depthmap_messages()
{
    IPC_RETURN_TYPE err;

    err = IPC_defineMsg(CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_NAME, IPC_VARIABLE_LENGTH, CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_NAME);
    return err;
}

IPC_RETURN_TYPE
carmen_zed_camera_sensor_publish_stereoimage_message(const carmen_zed_camera_sensor_stereoimage_message *message)
{
    IPC_RETURN_TYPE err;

    err = IPC_publishData(CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_NAME,(carmen_zed_camera_sensor_stereoimage_message *)message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_ZED_CAMERA_SENSOR_STEREOIMAGE_NAME);

    return err;
}

IPC_RETURN_TYPE
carmen_zed_camera_sensor_publish_depthmap_message(const carmen_zed_camera_sensor_depthmap_message *message)
{
    IPC_RETURN_TYPE err;

    err = IPC_publishData(CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_NAME,(carmen_zed_camera_sensor_depthmap_message *)message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_ZED_CAMERA_SENSOR_DEPTHMAP_NAME);

    return err;
}