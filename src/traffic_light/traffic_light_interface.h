#include <carmen/traffic_light_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

    IPC_RETURN_TYPE
    carmen_traffic_light_define_messages(int camera);

    char *
    carmen_traffic_light_message_name(int camera);

    void
    carmen_traffic_light_subscribe(int camera,
            carmen_traffic_light_message *traffic_light_message,
            carmen_handler_t handler,
            carmen_subscribe_t subscribe_how);

    void
    carmen_traffic_light_unsubscribe(int camera, carmen_handler_t handler);

    IPC_RETURN_TYPE
    carmen_traffic_light_publish_message(int camera,
            carmen_traffic_light_message *message);

    IPC_RETURN_TYPE
    carmen_mapping_traffic_light_define_messages();

    char *
    carmen_mapping_traffic_light_message_name();

    void
    carmen_mapping_traffic_light_subscribe(
            carmen_mapping_traffic_light_message *mapping_traffic_light_message,
            carmen_handler_t handler,
            carmen_subscribe_t subscribe_how);

    void
    carmen_mapping_traffic_light_unsubscribe(carmen_handler_t handler);

    IPC_RETURN_TYPE
    carmen_mapping_traffic_light_publish_message(carmen_mapping_traffic_light_message *message);


#ifdef __cplusplus
}
#endif