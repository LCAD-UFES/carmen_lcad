
#ifndef CARGO_INTERFACE_H
#define CARGO_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include "cargo_messages.h"

void
carmen_cargo_subscribe_cargos_message(carmen_cargo_cargos_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_cargo_publish_cargos_message(carmen_cargo_cargos_message *msg, double timestamp);

#ifdef __cplusplus
}
#endif

#endif
