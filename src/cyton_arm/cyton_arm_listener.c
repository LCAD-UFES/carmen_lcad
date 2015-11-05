#include <carmen/carmen.h>
#include <carmen/cyton_arm_messages.h>
#include <carmen/cyton_arm_interface.h>

carmen_cyton_arm_state_message s;

void state_handler()
{
	int aux;

	printf("X: %f \t Y: %f \t Z:%f\n", s.x, s.y, s.z);
	printf("Gripper: %f\n", s.gripper);
	printf("Arm busy: %d\n", s.arm_busy);
	
	printf("Number of joints: %d\t Vector Size: %ld\n", s.num_joints, sizeof(s.joint_angles)/sizeof(double));

	if(s.joint_angles != NULL)
	{
		//printf("Joint angles ->\t");
		for(aux = 0; aux < s.num_joints; aux++)
		{
			printf("%f ", s.joint_angles[aux]);
		}
		printf("\n");
	}

	printf("Timestamp: %f\n", s.timestamp);
	printf("Host: %s\n", s.host);
}

int main(int argc, char** argv)
{
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	carmen_cyton_arm_subscribe_state_message(&s,
					(carmen_handler_t)state_handler,
					CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}
