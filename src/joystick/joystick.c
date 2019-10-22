#include <carmen/carmen.h>
#include <carmen/joyctrl.h>
#include "joystick_interface.h"

static carmen_joystick_type joystick;

void sig_handler(int x)
{
  if(x == SIGINT) {
    carmen_close_joystick(&joystick);
    carmen_ipc_disconnect();
    printf("Disconnected from robot.\n");
    exit(0);
  }
}

void read_parameters(int argc, char **argv)
{
  int num_items;

  carmen_param_t param_list[] = {
  };

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);
}

int main(int argc, char **argv)
{
  double f_timestamp;
  int i;

  carmen_joystick_status_message message;

  message.axes = (int *) calloc (9, sizeof(int));
  message.buttons = (int *) calloc (12, sizeof(int));

  message.nb_axes = 9;
  message.nb_buttons = 12;

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);
  if (carmen_initialize_joystick(&joystick) < 0)
    carmen_die("Erorr: could not find joystick.\n");

  read_parameters(argc, argv);

  carmen_joystick_define_messages();

  signal(SIGINT, sig_handler);

  f_timestamp = carmen_get_time();
  while(1) {
    carmen_ipc_sleep(0.05);
    if(carmen_get_joystick_state(&joystick) >= 0) {
    	for(i = 0; i < message.nb_buttons; i++)
    	{

			message.buttons[i] = joystick.buttons[i];
			if(i < 9)
			{
				message.axes[i] = joystick.axes[i];
				printf("%d\n",joystick.axes[i]);
			}

    	}

    	message.timestamp = f_timestamp;
    	message.host = carmen_get_host();
    }

	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_JOYSTICK_STATUS_MESSAGE_NAME, &message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_JOYSTICK_STATUS_MESSAGE_NAME);

  }
  sig_handler(SIGINT);
  return 0;
}


