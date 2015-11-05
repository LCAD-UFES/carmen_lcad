 
#ifndef CARMEN_FUSED_ODOMETRY_INTERFACE_H
#define CARMEN_FUSED_ODOMETRY_INTERFACE_H

#include "fused_odometry_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_fused_odometry_subscribe_fused_odometry_message(	carmen_fused_odometry_message *fused_odometry_message,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how);

void
carmen_fused_odometry_unsubscribe_fused_odometry_message(carmen_handler_t handler);

void
carmen_fused_odometry_subscribe_fused_odometry_particle_message(	carmen_fused_odometry_particle_message *fused_odometry_paticle_message,
									carmen_handler_t handler,
									carmen_subscribe_t subscribe_how);

void
carmen_fused_odometry_unsubscribe_fused_odometry_particle_message(carmen_handler_t handler);

void
carmen_fused_odometry_publish_message(carmen_fused_odometry_message *message);

void
carmen_fused_odometry_publish_particles(carmen_fused_odometry_particle_message *message);

#ifdef __cplusplus
}
#endif

#endif
