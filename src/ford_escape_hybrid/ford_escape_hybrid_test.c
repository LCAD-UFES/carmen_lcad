#include <carmen/carmen.h>


carmen_base_ackerman_odometry_message odometry;


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_robot_and_trailer_motion_command_t motion_command;

	motion_command.time = 0.5;

	carmen_base_ackerman_subscribe_odometry_message(&odometry, NULL, CARMEN_SUBSCRIBE_LATEST);

	while(1)
	{
		printf("Escolha velocidade e phi em graus (v phi time): ");
		scanf("%lf %lf %lf", &motion_command.v, &motion_command.phi, &motion_command.time);
		motion_command.phi = carmen_degrees_to_radians(motion_command.phi);
		while(1)
		{
			carmen_ipc_sleep(0);
			printf("\nEstado atual: v=%f phi=%f\n", odometry.v, carmen_radians_to_degrees(odometry.phi));
			printf("Enviando comando, v=%f, phi=%f, time=%f\n",
					motion_command.v, carmen_radians_to_degrees(motion_command.phi), motion_command.time);
			carmen_base_ackerman_publish_motion_command(&motion_command, 1, carmen_get_time());
			carmen_ipc_sleep(0);
			printf("Enviar novamente (y, n)?[y]: ");
			if (getchar_unlocked() == 'n')
				break;
		}
	}


	carmen_ipc_disconnect();
	return 0;
}


