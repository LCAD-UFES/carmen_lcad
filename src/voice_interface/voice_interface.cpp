#include <Python.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include "voice_interface.h"


using namespace std;

PyObject *python_module_name, *python_module, *python_listen_function, *python_speak_function, *python_function_arguments;


char *
init_voice()
{
	Py_Initialize();
	python_module_name = PyUnicode_FromString((char *) "listen_speak");
	python_module = PyImport_Import(python_module_name);

	if (python_module == NULL)
	{
		Py_DECREF(python_module_name);
		Py_Finalize();
		return ((char *) "Error: The python_module could not be loaded.\n");
	}

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

	return (NULL); // OK
}


void 
finalize_voice()
{
	Py_XDECREF(python_speak_function);
	Py_XDECREF(python_listen_function);
	Py_DECREF(python_module);
	Py_Finalize();
}


int
speak(char *speech, char *speech_file_name)
{
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
	const char *listen_function_output = PyBytes_AS_STRING(python_listen_function_output);
	strncpy(listened_string, listen_function_output, MAX_LISTENDED_STRING_SIZE - 1);
	Py_XDECREF(python_listen_function_output);
}
