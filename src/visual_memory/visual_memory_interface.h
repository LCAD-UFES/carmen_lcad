#include <carmen/visual_memory_messages.h>

#include "ipc_wrapper.h"

#ifndef CARMEN_VISUAL_MEMORY_INTERFACE_H
#define CARMEN_VISUAL_MEMORY_INTERFACE_H

#ifdef __cplusplus
extern "C"
{
#endif


    void
    carmen_visual_memory_subscribe_train(carmen_visual_memory_message *visual_memory_message,
                                         carmen_handler_t handler,
                                         carmen_subscribe_t subscribe_how);

    void
    carmen_visual_memory_unsubscribe_train(carmen_handler_t handler);

    IPC_RETURN_TYPE
    carmen_visual_memory_define_message_train(void);

    IPC_RETURN_TYPE
    carmen_visual_memory_define_message_output(void);   


#ifdef __cplusplus
}
#endif

#endif
