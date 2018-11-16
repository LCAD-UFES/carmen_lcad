#include <stdio.h>
#include <iostream>
#include <carmen/carmen.h>
#include <carmen/voice_interface_interface.h>
#include "voice_interface.h"
#include "porcupine_keyword.h"

using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_voice_interface_can_line_message(char *can_line)
{
//	carmen_voice_interface_can_line_message message;

	can_line[strlen(can_line) - 1] = '\0'; // Apaga o '\n' no fim da string

	FILE *caco = fopen("voice_interface.txt", "a");
	fprintf(caco, "%lf can_line %s\n", carmen_get_time(), can_line);
	fflush(caco);
	fclose(caco);

//	message.can_line = can_line;
//	message.timestamp = carmen_get_time();
//	message.host = carmen_get_host();

//	carmen_voice_interface_publish_can_line_message(&message);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		printf("voice interface: disconnected.\n");
		finalize_voice();
		finalize_porcupine();

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
voice_interface_define_messages()
{
	carmen_voice_interface_define_can_line_message();
}


void
carmen_voice_interface_initialize()
{
	char *voice_interface_error = init_voice();
	if (voice_interface_error != NULL)
	{
		printf("Error: could not initialize the voice interface.\n%s\n", voice_interface_error);
		exit(1);
	}

	char *porcupine_error = initialize_porcupine();
	if (porcupine_error != NULL)
	{
		printf("Error: could not initialize porcupine.\n%s\n", porcupine_error);
		exit(1);
	}
}


int
main (int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
//	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	voice_interface_define_messages();
	carmen_voice_interface_initialize();

	while (true)
	{
		char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Oi.");
		if (voice_interface_speak_error)
			printf("%s \n", voice_interface_speak_error);

		hotword_detection();
		system("mpg123 computerbeep_4.mp3");
		const char *voice_command = carmen_voice_interface_listen();
		if (voice_command)
			printf("%s \n", voice_command);
		carmen_ipc_sleep(0.02);
	}

	return (0);
}
