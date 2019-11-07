
#include <carmen/carmen.h>


void arm_state_handler(carmen_arm_state_message *arm)
{
  int i;

  printf("[ ");
  for (i = 0; i < arm->num_joints; i++)
    printf("%f ", carmen_radians_to_degrees(arm->joint_angles[i]));
  printf("%f ] ;\n", arm->timestamp);
}

void shutdown_module(int sig)
{
  if(sig == SIGINT) {
    carmen_ipc_disconnect();
    printf("]\n");
    fprintf(stderr, "\nDisconnecting.\n");
    exit(0);
  }
}

int main(int argc, char *argv[]) {

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);	

  printf("[ ");

  carmen_arm_subscribe_state_message(NULL, (carmen_handler_t)
				     arm_state_handler, 
				     CARMEN_SUBSCRIBE_ALL);

  signal(SIGINT, shutdown_module);
  carmen_ipc_dispatch();

  return 0;
}
