 
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

/*
 * Define as mensagens de fused_odometry neste processo. Nao precisa ser chamada a mao:
 * os subscribe/publish desta interface ja' chamam. Existe para quebrar a dependencia de
 * ordem de subida -- IPC_subscribe exige a mensagem definida, e um assinante que sobe
 * antes do publicador falharia EM SILENCIO. Idempotente.
 */
void
carmen_fused_odometry_define_messages(void);

void
carmen_fused_odometry_publish_particles(carmen_fused_odometry_particle_message *message);

#ifdef __cplusplus
}
#endif

#endif
