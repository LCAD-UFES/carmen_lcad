#include <carmen/carmen.h>
#include <carmen/cyton_arm_messages.h>
#include <carmen/cyton_arm_interface.h>

carmen_cyton_arm_point_command_message p;
carmen_cyton_arm_joint_command_message j;
carmen_cyton_arm_state_message arm_state;

int got_state = 0;

void state_handler()
{
	got_state = 1;
	//Handle the state if needed
}

int main(int argc, char** argv)
{
	int aux, i;
	double x, y, z;
	IPC_RETURN_TYPE err;
	FILE *file = fopen("results.txt", "r");

	if (!file)
	{
		exit(1);
	}

	//carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	carmen_cyton_arm_subscribe_state_message(&arm_state,
					(carmen_handler_t)state_handler,
					CARMEN_SUBSCRIBE_LATEST);
	
	if (argc != 2)
	{
		carmen_die("\nUsage: ./cyton_arm_loop <param> in which param can be:\n\t- 1 to Point command loop\n\t- 2 to Joint command loop\n");
	}

	aux = atoi(argv[1]);

	switch (aux)
	{
		case 1:
			x = y = 0.23;
			for (x = 0.23; x > 0.0; x = x - 0.02)
			{
				for (y = 0.23; y > 0.0; y = y - 0.02)
				{
					for (z = -0.23; z < 0.23; z = z + 0.02)
					{
						p.x = x;
						p.y = y;
						p.z = z;
						p.gripper = 0;

						p.timestamp = carmen_get_time();
						p.host = carmen_get_host();
						
						while(arm_state.arm_busy)
							;

						err = IPC_publishData(CARMEN_CYTON_ARM_POINT_COMMAND_NAME, &p);
						carmen_test_ipc_exit(err, "Could not publish", CARMEN_CYTON_ARM_POINT_COMMAND_NAME);
					}
				}
			}
			break;

		case 2:
			while (!feof(file))
			{
				j.num_joints = 8;

				j.joint_angles = (double*)malloc(j.num_joints*sizeof(double));

				if (!j.joint_angles)
				{
					printf("Unable to allocate enough memory to the joint message.\n");
					exit(1);
				}

				fscanf(file, "%lf", &j.joint_angles[0]);
				fscanf(file, "%lf", &j.joint_angles[1]);
				fscanf(file, "%lf", &j.joint_angles[2]);
				fscanf(file, "%lf", &j.joint_angles[3]);
				fscanf(file, "%lf", &j.joint_angles[4]);
				fscanf(file, "%lf", &j.joint_angles[5]);
				fscanf(file, "%lf", &j.joint_angles[6]);
				fscanf(file, "%lf", &j.joint_angles[7]);

				j.timestamp = carmen_get_time();
				j.host = carmen_get_host();

				for (i = 0; i < 8; i++)
				{
					printf("%lf ", j.joint_angles[i]);
				}
				printf("\n");

				while (arm_state.arm_busy)
				{
					carmen_ipc_sleep(0.1);
				}
				
				err = IPC_publishData(CARMEN_CYTON_ARM_JOINT_COMMAND_NAME, &j);
				carmen_test_ipc_exit(err, "Could not publish", CARMEN_CYTON_ARM_JOINT_COMMAND_NAME);
				
				free(j.joint_angles);
				//arm_state.arm_busy = 1; //While state is not updated, sets arm_busy to 1
				carmen_ipc_sleep(0.2);
			}
			break;
	}

	carmen_ipc_dispatch();

	return 0;
}
