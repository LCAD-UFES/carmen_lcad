#ifndef TEST_H
#define TEST_H

typedef struct {
	double timestamp;
	char text[1024];
} carmen_test_ipc_message;

#define CARMEN_TEST_IPC_NAME "carmen_test_ipc"
#define CARMEN_TEST_IPC_FMT  "{double,[char:1024]}"

#endif
