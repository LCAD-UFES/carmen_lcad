
#ifndef VOICE_INTERFACE_H_
#define VOICE_INTERFACE_H_

#include <Python.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
* Initiate Python interpreter and verify module
*/
char *init_voice();

/*
* Finalize Python interpreter and "drecrement" Python variables
*/
void finalize_voice();

/*
* Transforms current text in audio
*/
int speak(char *text, char *speech_file_name);

/*
* Enable Microfone to output a text from an audio
*/
const char* listen();

#endif /* _VOICE_INTERFACE_H_ */
