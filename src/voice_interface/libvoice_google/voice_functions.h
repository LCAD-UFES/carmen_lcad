
#ifndef LIB_VOICE_FUNCTIONS_H_
#define LIB_VOICE_FUNCTIONS_H_

#include <Python.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
* Initiate Python interpreter and verify module
*/
void init_voice();

/*
* Finalize Python interpreter and "drecrement" Python variables
*/
void finalize_voice();

/*
* Transforms current text in audio
*/
void speak(char* text);

/*
* Enable Microfone to output a text from an audio
*/
const char* listen();

#endif /* _VOICE_FUNCTIONS_H_ */
