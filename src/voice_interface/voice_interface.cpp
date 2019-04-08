#include <Python.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include "voice_interface.h"


using namespace std;

PyObject *python_module, *python_language_function, *python_listen_function, *python_speak_function;


char *
init_voice(/*char *language_code*/)
{
	static bool already_initialized = false;

	if (already_initialized)
		return (NULL);

	Py_Initialize();

	PyObject *sysPath = PySys_GetObject((char *) "path");

	char *carmen_dir = getenv("CARMEN_HOME");
	if (carmen_dir == NULL)
		return ((char *) "CARMEN_HOME not defined in init_voice()\n");
	char voice_interface_path[1024];
	strcpy(voice_interface_path, carmen_dir);
	strcat(voice_interface_path, "/src/voice_interface");

	PyObject *python_program_path = PyUnicode_FromString(voice_interface_path);
	PyList_Append(sysPath, python_program_path);
	Py_DECREF(python_program_path);

	PyObject *python_module_name = PyUnicode_FromString((char *) "listen_speak");

	PyObject *python_module = PyImport_Import(python_module_name);
	Py_DECREF(python_module_name);

	if (python_module == NULL)
	{
		Py_Finalize();
		return ((char *) "Error: The python_module could not be loaded.\n");
	}

	/*python_language_function = PyObject_GetAttrString(python_module, (char *) "language");
	if (python_language_function == NULL || !PyCallable_Check(python_language_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		return ((char *) "Error: Could not load the python_module language function.\n");
	}

	set_language(language_code);*/

	python_speak_function = PyObject_GetAttrString(python_module, (char *) "speak");
	if (python_speak_function == NULL || !PyCallable_Check(python_speak_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		return ((char *) "Error: Could not load the python_module speak function.\n");
	}

	python_listen_function = PyObject_GetAttrString(python_module, (char *) "listen");
	if (python_listen_function == NULL || !PyCallable_Check(python_listen_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		return ((char *) "Error: Could not load the python_module listen function.\n");
	}

	already_initialized = true;

	return (NULL); // OK
}

void
finalize_voice()
{
	Py_XDECREF(python_language_function);
	Py_XDECREF(python_speak_function);
	Py_XDECREF(python_listen_function);
	Py_DECREF(python_module);
	Py_Finalize();
}
/*
void
set_language(char *language_to_set)
{

	PyObject *python_function_arguments = Py_BuildValue("s", language_to_set);
	PyObject *python_language_function_output = PyObject_CallObject(python_language_function, python_function_arguments);
	Py_DECREF(python_function_arguments);
	//Py_DECREF(python_language_function_output);
}
 */

int
speak(char *speech, char *speech_file_name)
{
	// Saves the speech fine in $CARMEN_HOME/data/voice_interface_speechs/ (see listen_speak.py))
	PyObject *python_function_arguments = Py_BuildValue("(ss)", speech, speech_file_name);
	PyObject *python_speak_function_output = PyObject_CallObject(python_speak_function, python_function_arguments);
	Py_DECREF(python_function_arguments);
	Py_DECREF(python_speak_function_output);

	return (0); // OK
}


void
listen(char *listened_string)
{
	PyObject *python_listen_function_output = PyObject_CallFunction(python_listen_function, NULL);
	if (python_listen_function_output)
	{
		const char *listen_function_output = PyBytes_AS_STRING(python_listen_function_output);
		strncpy(listened_string, listen_function_output, MAX_LISTENDED_STRING_SIZE - 1);
		Py_XDECREF(python_listen_function_output);
	}
	else
		listened_string[0] = '\0';
}
