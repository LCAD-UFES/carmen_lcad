
#ifndef _CARMEN_RDDF_INTERFACE_H_
#define _CARMEN_RDDF_INTERFACE_H_

#include <carmen/carmen.h>
#include <carmen/rddf_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

    char* rddf_get_annotation_description_by_type(int type);

    void carmen_rddf_subscribe_road_profile_message(carmen_rddf_road_profile_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
    void carmen_rddf_subscribe_end_point_message(carmen_rddf_end_point_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
    void carmen_rddf_subscribe_nearest_waypoint_message(carmen_rddf_nearest_waypoint_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
    void carmen_rddf_subscribe_waypoints_around_end_point_message(carmen_rddf_waypoints_around_end_point_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
    void carmen_rddf_subscribe_nearest_waypoint_confirmation_message(carmen_rddf_nearest_waypoint_confirmation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
    void carmen_rddf_subscribe_add_annotation_message(carmen_rddf_add_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
    void carmen_rddf_subscribe_annotation_message(carmen_rddf_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

    void carmen_rddf_unsubscribe_road_profile_message(carmen_handler_t handler);
    void carmen_rddf_unsubscribe_end_point_message(carmen_handler_t handler);
    void carmen_rddf_unsubscribe_nearest_waypoint_message(carmen_handler_t handler);
    void carmen_rddf_unsubscribe_waypoints_around_end_point_message(carmen_handler_t handler);
    void carmen_rddf_unsubscribe_nearest_waypoint_confirmation_message(carmen_handler_t handler);
    void carmen_rddf_unsubscribe_add_annotation_message(carmen_handler_t handler);
    void carmen_rddf_unsubscribe_annotation_message(carmen_handler_t handler);

    void carmen_rddf_define_messages();

    void carmen_rddf_publish_end_point_message(int number_of_poses_desired, carmen_point_t point);
    void carmen_rddf_publish_nearest_waypoint_message(int number_of_poses_desired, carmen_point_t point);
    void carmen_rddf_publish_road_profile_message(carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int num_poses, int num_poses_back, int *annotations);
    void carmen_rddf_publish_nearest_waypoint_confirmation_message(carmen_point_t point);
    void carmen_rddf_publish_road_profile_around_end_point_message(carmen_ackerman_traj_point_t *poses_around_end_point, int num_poses);
    void carmen_rddf_publish_add_annotation_message(carmen_vector_3D_t annotation_point, double orientation, char *annotation_description, int annotation_type, int annotation_code);
    void carmen_rddf_publish_annotation_message(carmen_vector_3D_t annotation_point, double orientation, char *annotation_description, int annotation_type, int annotation_code);

#ifdef __cplusplus
}
#endif

#endif

// @}

