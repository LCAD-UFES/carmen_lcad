#include <QtGui/QApplication>
#include "skeleton_module_view.h"

QApplication* a = NULL;
skeleton_module_view* w = NULL;

void upkeymessage_handler(carmen_skeleton_module_filter_upkeymessage_message* message)
{
    if(w != NULL)
            w->update_char_flow_image(message->up_caracter);
}

void shutdown_module(int signo)
{
  if(signo == SIGINT)
  {
     carmen_ipc_disconnect();
     printf("skeleton_module_view: disconnected.\n");
     exit(0);
  }
}

int main(int argc, char *argv[])
{
    a = new QApplication(argc, argv);
    w = new skeleton_module_view();

    /* Do Carmen Initialization*/
    carmen_ipc_initialize(argc, argv);

    /* Check the param server version */
    carmen_param_check_version(argv[0]);

    /* Register shutdown cleaner handler */
    signal(SIGINT, shutdown_module);

    /* Register Carmen Callbacks to Qt interface */
    carmen_graphics_update_ipc_callbacks_qt(w, SLOT(updateIPC(int)));

    /* Define published messages by your module */
    carmen_skeleton_module_filter_define_messages();

    /* Subscribe to sensor messages */
    carmen_skeleton_module_filter_subscribe_upkeymessage(NULL, (carmen_handler_t) upkeymessage_handler, CARMEN_SUBSCRIBE_LATEST);

    w->show();
    return a->exec();
}
