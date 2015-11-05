#include <unistd.h>
#include <sys/time.h>
#include "ipc.h"

#define CARMEN_TEST_IPC_NAME "carmen_test_ipc"
#define CARMEN_TEST_IPC_FMT  "{double,[char:1024]}"

typedef struct {
  double timestamp;
  char text[1024];
} carmen_test_ipc_message;

IPC_CONTEXT_PTR *context;
int num_contexts = 0;
int cur_context = 0;

char *module_name;

void x_ipcRegisterExitProc(void (*)(void));
void reconnect(void);
IPC_RETURN_TYPE connect_ipc(void);

static void msgHandler (MSG_INSTANCE msgRef, BYTE_ARRAY callData,
			void *clientData)
{
  FORMATTER_PTR formatter;
  carmen_test_ipc_message msg;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  IPC_unmarshallData(formatter, callData, &msg,
                           sizeof(carmen_test_ipc_message));
  IPC_freeByteArray(callData);
  
#if (defined(__x86_64__))
  fprintf(stderr, "Got message from client %ld : time %f\n",
	  (long)clientData, msg.timestamp);
#else
  fprintf(stderr, "Got message from client %d : time %f\n",
	  (int)clientData, msg.timestamp);
#endif
}

IPC_RETURN_TYPE connect_ipc(void)
{
  IPC_RETURN_TYPE err;

  IPC_setVerbosity(IPC_Silent);

  err = IPC_connect(module_name);
  if (err != IPC_OK)
    return err;

  x_ipcRegisterExitProc(reconnect);

  IPC_subscribe(CARMEN_TEST_IPC_NAME, msgHandler, NULL);
  IPC_setMsgQueueLength(CARMEN_TEST_IPC_NAME, 1); 
  
  return err;
}

void reconnect(void)
{
  IPC_RETURN_TYPE err;

  do {
    fprintf(stderr, "IPC died. Reconnecting...\n");
    if (IPC_isConnected())
      IPC_disconnect();
    err = connect_ipc();
    if (err == IPC_OK)
      fprintf(stderr, "Reconnected...\n");
  } while (err == IPC_Error);
}

int main (int argc, char *argv[])
{
  module_name = argv[0];

  connect_ipc();

  IPC_dispatch();

  return 0;
}
