#ifndef CARMEN_VISUAL_SEARCH_THIN_HANDLERS_H
#define CARMEN_VISUAL_SEARCH_THIN_HANDLERS_H

#include <carmen/carmen.h>
#include "visual_search_thin_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void	training_message_handler(carmen_visual_search_thin_train_message *);
void	test_message_handler(carmen_visual_search_thin_test_message *);
void 	query_training_message_handler(MSG_INSTANCE, BYTE_ARRAY, void *);
void 	query_test_message_handler(MSG_INSTANCE, BYTE_ARRAY, void *);

#ifdef __cplusplus
}
#endif

#endif /* CARMEN_VISUAL_SEARCH_THIN_HANDLERS_H */
