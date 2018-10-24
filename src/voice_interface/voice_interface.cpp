#include "Python.h"
#include <stdio.h>
#include <iostream>

using namespace std;

PyObject *python_module_name, *python_module, *python_listen_function, *python_speak_function, *python_function_arguments;


char *
init_voice()
{
	Py_Initialize();
	python_module_name = PyString_FromString("listen_speak");

	python_module = PyImport_Import(python_module_name);
	Py_DECREF(python_module_name);

	if (python_module == NULL)
	{
		Py_Finalize();
		return ("Error: the module could not be loaded.\n");
	}

	python_speak_function = PyObject_GetAttrString(python_module, "speak");

	if (python_speak_function == NULL || !PyCallable_Check(python_speak_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		return ("Error: Could not load SPEAK function.\n");
	}

	python_listen_function = PyObject_GetAttrString(python_module, "listen");

	if (python_listen_function == NULL || !PyCallable_Check(python_listen_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		return ("Error: Could not load LISTEN function.\n");
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


const char* 
listen()
{
	PyObject *python_listen_function_output = PyObject_CallFunction(python_listen_function, NULL);
	const char *listen_function_output = PyString_AsString(python_listen_function_output);
	const char *words = listen_function_output;
	Py_DECREF(python_listen_function_output);
	return words;
}
