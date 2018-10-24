#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "voice_functions.h"


void
create_new_audio_file(FILE *list_of_speechs_file, char *inputed_string, char *last_audio_name_used)
{
	char command[1024], default_name_speech[6], last_number_used[4];
	int new_number;

	strncpy(default_name_speech, last_audio_name_used, 6);
	strncpy(last_number_used, last_audio_name_used+6, 4);

	new_number = atoi(last_number_used);
	new_number++;

	sprintf(default_name_speech, "%s%.4d", default_name_speech, new_number);

	init_voice();
	speak((char*)inputed_string);
	finalize_voice();

	sprintf(command, "mv voice_sample.wav ./%s.wav", default_name_speech);
	system(command);

	fprintf(list_of_speechs_file, "%s %s\n", default_name_speech, inputed_string);

}

void
setting_voice_interface_list(char *input_string)
{
	FILE *voice_interface_list_file;
	char *audio_file_name, speakers_command[1024], *audio_file_string, *audio_input_string, *full_file_line;
	int found_it = 0, is_file_empty = 0;

	voice_interface_list_file = fopen("list_of_speechs.txt", "a+");

	is_file_empty = getc(voice_interface_list_file);

	//size_input_string = strlen(input_string);
	audio_file_string = (char*)malloc(100*sizeof(char));
	audio_file_name = (char*)malloc(11*sizeof(char));
	full_file_line = (char*)malloc(100*sizeof(char));
	audio_input_string = (char*)malloc(100*sizeof(char));

	strcpy(audio_input_string, input_string);
	audio_input_string[strlen(audio_input_string)] = '\n';

	if (is_file_empty != EOF)
	{
		fseek(voice_interface_list_file, 0, SEEK_SET);

		do
		{
			fgets(full_file_line, 100, voice_interface_list_file);
			strncpy(audio_file_name, full_file_line, 10);
			//char space = getc(voice_interface_list_file);
			strncpy(audio_file_string, full_file_line+11, 100);

			if (strcmp(audio_file_string, audio_input_string) == 0)
			{
				sprintf(speakers_command, "aplay %s.wav", audio_file_name);
				system(speakers_command);
				found_it = 1;
			}
		} while (!feof(voice_interface_list_file) && (found_it == 0));

		if (found_it == 0){
			create_new_audio_file(voice_interface_list_file, input_string, audio_file_name);
		}
	}
	else
	{
		create_new_audio_file(voice_interface_list_file, input_string, (char*)"Speech0000");
	}

	free(audio_file_string);
	free(audio_input_string);
	free(full_file_line);
	free(audio_file_name);
	fclose(voice_interface_list_file);
}

int
main()
{
	char nome[100] = "Atenção! Alguns módulos estão instáveis!";
	setting_voice_interface_list((char*)nome);

	return 0;
}
