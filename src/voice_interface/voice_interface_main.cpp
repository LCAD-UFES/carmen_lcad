#include <stdio.h>
#include <iostream>
#include <carmen/carmen.h>
#include <carmen/voice_interface_interface.h>
#include <alsa/asoundlib.h>
#include "voice_interface.h"
#include "porcupine_keyword.h"

using namespace std;

extern snd_pcm_t* capture_handle;


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


bool
check_command(char *voice_command, char *command_template)
{
	return (true);
}


void
execute_command(char *voice_command)
{
	if (voice_command)
	{
		printf("\nVoice command: %s \n", voice_command);
		if (strcmp(voice_command, "timeout") == 0)
		{
			char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Desculpe... Não consegui captar o comando.");
			if (voice_interface_speak_error)
				printf("%s \n", voice_interface_speak_error);
		}
		else if (check_command(voice_command, (char *) "seguir curso"))
		{
			printf("\Command detected: %s \n\n", "seguir curso");
			carmen_navigator_ackerman_go();
		}
	}
}


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

//	char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Oi Alberto!");
//	if (voice_interface_speak_error)
//		printf("%s \n", voice_interface_speak_error);

	printf("Awaiting hotword\n");
	while (true)
	{
		int hotword_detection_result = hotword_detection();
		if (hotword_detection_result == 1) // hotword detected
		{
			snd_pcm_drop(capture_handle);

			printf("Hotword detected\n");

			carmen_ipc_sleep(0.1); // Necessario para reconectar com o audio para tocar o som abaixo.
			system("mpg123 $CARMEN_HOME/data/voice_interface_hotword_data/computerbeep_4.mp3");

			printf("Awaiting for command\n\n");
			char *voice_command = carmen_voice_interface_listen();
			execute_command(voice_command);

			snd_pcm_prepare(capture_handle);
			snd_pcm_start(capture_handle);

			printf("Awaiting hotword\n");
		}
		else if (hotword_detection_result == 2) // error
			printf ("Error in hotword detection\n");

		carmen_ipc_sleep(0.0);
	}

	return (0);
}
