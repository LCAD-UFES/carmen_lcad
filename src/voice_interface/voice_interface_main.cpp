#include <stdio.h>
#include <carmen/carmen.h>
#include <carmen/voice_interface_interface.h>



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

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
voice_interface_initialize()
{
	carmen_voice_interface_define_can_line_message();
}


int
main (int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	voice_interface_initialize();

	char *voice_interface_error = carmen_voice_interface_speak((char *) "Só alegria!");
	if (voice_interface_error)
		printf("%s", voice_interface_error);

	return (0);
}
