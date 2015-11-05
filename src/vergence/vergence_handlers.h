#ifndef CARMEN_VERGENCE_HANDLERS_H
#define CARMEN_VERGENCE_HANDLERS_H

#include <carmen/carmen.h>
#include "vergence_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void 	query_vergence_train_message_handler(MSG_INSTANCE, BYTE_ARRAY, void *);

void 	query_vergence_test_message_handler(MSG_INSTANCE, BYTE_ARRAY, void *);

#ifdef __cplusplus
}
#endif

#endif /* CARMEN_VERGENCE_HANDLERS_H */
