#include "Python.h"
#include <stdio.h>
#include "voice_functions.h"
#include <iostream>

using namespace std;

PyObject *pName, *pModule, *pFuncL, *pFuncS, *pArgs;


void 
init_voice()
{
	Py_Initialize();
	pName = PyString_FromString("listen_speak");

	//pName = PyUnicode_DecodeFSDefault("listen_speak");	
	pModule = PyImport_Import(pName);
	Py_DECREF(pName);

	if (pModule == NULL)
	{
		Py_Finalize();
		exit(printf("Error: the module could not be loaded.\n"));
	}

	pFuncS = PyObject_GetAttrString(pModule, "speak");

	if (pFuncS == NULL || !PyCallable_Check(pFuncS))
	{
		Py_DECREF(pModule);
		Py_Finalize();
		exit(printf("Error: Could not load SPEAK function.\n"));
	}

	pFuncL = PyObject_GetAttrString(pModule, "listen");

	if (pFuncL == NULL || !PyCallable_Check(pFuncL))
	{
		Py_DECREF(pModule);
		Py_Finalize();
		exit(printf("Error: Could not load LISTEN function.\n"));
	}
}


void 
finalize_voice()
{
	Py_XDECREF(pFuncS);
	Py_XDECREF(pFuncL);
	Py_DECREF(pModule);
	Py_Finalize();
}


void 
speak(char* text)
{
	pArgs = Py_BuildValue("(z)", text); //generic function, create objects from C values by a format string
	PyObject *need_output = PyObject_CallObject(pFuncS, pArgs);
	Py_DECREF(pArgs);
	Py_DECREF(need_output);
}


const char* 
listen()
{
	PyObject *py_Output = PyObject_CallFunction(pFuncL, NULL);
	const char *listen_output = PyString_AsString(py_Output);
	const char *words = listen_output;
	Py_DECREF(py_Output);
	return words;
}
