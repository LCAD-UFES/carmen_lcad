
#ifndef SRC_VOICE_INTERFACE_LIBVOICE_GOOGLE_SETTING_VOICE_LIST_H_
#define SRC_VOICE_INTERFACE_LIBVOICE_GOOGLE_SETTING_VOICE_LIST_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "voice_functions.h"

//#define LIST_OF_SPEECHS_PATH "$CARMEN_HOME/src/voice_interface/libvoice_google/list_of_speechs.txt"
//#define AUDIO_FILE_DESTINATION "$CARMEN_HOME/src/voice_interface/libvoice_google/"

void
create_new_audio_file(FILE *list_of_speechs_file, char *inputed_string, char *last_audio_name_used);

void
setting_voice_interface_list(char *input_string);

#endif /* SRC_VOICE_INTERFACE_LIBVOICE_GOOGLE_SETTING_VOICE_LIST_H_ */
