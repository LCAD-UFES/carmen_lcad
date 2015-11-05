#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "ipc.h"

typedef struct {
  double timestamp;
  char text[1024];
} carmen_test_ipc_message;

#define CARMEN_TEST_IPC_NAME "carmen_test_ipc"
#define CARMEN_TEST_IPC_FMT  "{double,[char:1024]}"

IPC_CONTEXT_PTR *context;
int num_contexts = 0;
int cur_context = 0;

void x_ipcRegisterExitProc(void (*)(void));

static void handle_exit(void)
{
  fprintf(stderr, "Caught exit on %d.\n", cur_context); 

  context[cur_context] = NULL;
  if (cur_context == num_contexts-1)
    cur_context = 0;
  else
    cur_context++;

  IPC_setContext(context[cur_context]);
  fprintf(stderr, "Set context to %d.\n", cur_context); 
  return;
}

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

int main(int argc, char *argv[])
{
  num_contexts = argc-1;
  context = (IPC_CONTEXT_PTR *)calloc(num_contexts, sizeof(IPC_CONTEXT_PTR));
  /* check_alloc checked */

  IPC_setVerbosity(IPC_Print_Warnings);
  
  for (cur_context = 0; cur_context < num_contexts; cur_context++) {
    IPC_connectModule(argv[0], argv[cur_context+1]);
    context[cur_context] = IPC_getContext();
#if (defined(__x86_64__))
    IPC_subscribe(CARMEN_TEST_IPC_NAME, msgHandler, (void *)(long)cur_context);
#else
    IPC_subscribe(CARMEN_TEST_IPC_NAME, msgHandler, (void *)cur_context);
#endif
    IPC_setMsgQueueLength(CARMEN_TEST_IPC_NAME, 1);
  }

  x_ipcRegisterExitProc(handle_exit);

  while (1) {
    for (cur_context = 0; cur_context < num_contexts; cur_context++) {
      fprintf(stderr, "chunk %d\n", cur_context);
      if (context[cur_context] != NULL) {
	IPC_setContext(context[cur_context]);
	IPC_listenClear(0);
	sleep(1);
      }
    } 
  }
   
  return 0;
}
