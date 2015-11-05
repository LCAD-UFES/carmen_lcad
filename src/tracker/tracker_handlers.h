#ifndef TRACKER_HANDLERS_H
#define TRACKER_HANDLERS_H

#include <carmen/carmen.h>
#include "tracker_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void	training_message_handler(carmen_tracker_train_message *);
void	test_message_handler(carmen_tracker_test_message *);
void 	query_training_message_handler(MSG_INSTANCE, BYTE_ARRAY, void *);
void 	query_test_message_handler(MSG_INSTANCE, BYTE_ARRAY, void *);

#ifdef __cplusplus
}
#endif

#endif /* CARMEN_TRACKER_HANDLERS_H */
