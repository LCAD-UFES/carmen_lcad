#include <carmen/vergence_messages.h>

#ifndef CARMEN_VERGENCE_INTERFACE_H
#define CARMEN_VERGENCE_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

IPC_RETURN_TYPE
carmen_vergence_define_message_train(void);

IPC_RETURN_TYPE
carmen_vergence_define_message_test(void);

IPC_RETURN_TYPE
carmen_vergence_define_message_output_test(void);

IPC_RETURN_TYPE
carmen_vergence_define_message_output_train(void);

IPC_RETURN_TYPE
carmen_vergence_define_message_query_train(void);

IPC_RETURN_TYPE
carmen_vergence_define_message_query_test(void);

void
carmen_vergence_subscribe_query_train(HANDLER_TYPE handler);

void
carmen_vergence_subscribe_query_test(HANDLER_TYPE handler);

carmen_vergence_train_output_message *
carmen_vergence_query_train_message(carmen_vergence_train_message *query, double timeout);

carmen_vergence_test_output_message *
carmen_vergence_query_test_message(carmen_vergence_test_message *query, double timeout);

#ifdef __cplusplus
}
#endif

#endif
