#include <unistd.h>
#include <sys/time.h>
#include "ipc.h"

typedef struct {
        double timestamp;
        char text[1024];
} carmen_test_ipc_message;

#define CARMEN_TEST_IPC_NAME "carmen_test_ipc"
#define CARMEN_TEST_IPC_FMT  "{double,[char:1024]}"

int main(int argc __attribute__ ((unused)),
	 char *argv[] __attribute__ ((unused)))
{
  carmen_test_ipc_message msg;
  struct timeval tv;
  
  IPC_connect(argv[0]);
  
  IPC_defineMsg(CARMEN_TEST_IPC_NAME, IPC_VARIABLE_LENGTH,
		CARMEN_TEST_IPC_FMT);
  
  while (1) {
    gettimeofday(&tv, NULL);
    msg.timestamp = tv.tv_sec + tv.tv_usec/1000000.0;
    IPC_publishData(CARMEN_TEST_IPC_NAME, &msg);
  }
}
