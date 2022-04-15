#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <numpy/arrayobject.h>
#include <iostream>
#include "libnewcrfs.h"
#include <stdlib.h> /* getenv */

#define NUMPY_IMPORT_ARRAY_RETVAL

PyObject *python_libnewcrfs_process_image_function;

void initialize_python_path_newcrfs()
{
	char *pyPath;
	char *pPath;
	char *newcrfsPath;
	pyPath = (char *)"PYTHONPATH=";
	pPath = getenv("CARMEN_HOME");
	newcrfsPath = (char *)"/src/deep_mapper/newcrfs";
	char *path = (char *)malloc(1 + strlen(pyPath) + strlen(pPath) + strlen(newcrfsPath));
	strcpy(path, pyPath);
	strcat(path, pPath);
	strcat(path, newcrfsPath);
	putenv(path);
}

void initialize_python_context_newcrfs()
{
	initialize_python_path_newcrfs();
	Py_Initialize();
	import_array();
	PyObject *python_module = PyImport_ImportModule("run_newcrfs");

	if (PyErr_Occurred())
		PyErr_Print();

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module run_newcrfs could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
	}

	if (PyErr_Occurred())
		PyErr_Print();

	PyObject *python_initialize_function = PyObject_GetAttrString(python_module, (char *)"initialize");

	if (PyErr_Occurred())
		PyErr_Print();
	if (python_initialize_function == NULL || !PyCallable_Check(python_initialize_function))
	{
		Py_Finalize();
		exit(printf("Error: Could not load the python_initialize_function.\n"));
	}
	PyObject *python_arguments = Py_BuildValue("()");

	if (PyErr_Occurred())
		PyErr_Print();

	PyObject_CallObject(python_initialize_function, python_arguments);

	if (PyErr_Occurred())
		PyErr_Print();

	// Py_DECREF(python_arguments);
	// Py_DECREF(python_initialize_function);

	python_libnewcrfs_process_image_function = PyObject_GetAttrString(python_module, (char *)"newcrfs_process_image");

	if (PyErr_Occurred())
		PyErr_Print();

	if (python_libnewcrfs_process_image_function == NULL || !PyCallable_Check(python_libnewcrfs_process_image_function))
	{
		// Py_DECREF(python_module);
		Py_Finalize();
		exit(printf("Error: Could not load the newcrfs_process_image.\n"));
	}

	if (PyErr_Occurred())
		PyErr_Print();

	printf("Success: Loaded newcrfs\n");
}

unsigned char *
libnewcrfs_process_image(int width, int height, unsigned char *image, int cut_param, int down_param)
{
	// printf("libnewcrfs_process_image\n");
	npy_intp dims[3] = {height, width, 3};
	PyObject *numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image);

	int time[1];
	time[0] = cut_param;
	npy_intp dimcut[1] = {1};

	PyObject *numpyCutParam = PyArray_SimpleNewFromData(1, dimcut, NPY_INT, &time[0]);
	int down[1];
	down[0] = down_param;
	PyObject *numpyDownParam = PyArray_SimpleNewFromData(1, dimcut, NPY_INT, &down[0]);

	if (PyErr_Occurred())
		PyErr_Print();
	PyArrayObject *python_result_array = (PyArrayObject *)PyObject_CallFunctionObjArgs(python_libnewcrfs_process_image_function, numpyArray, numpyCutParam, numpyDownParam, NULL);

	if (PyErr_Occurred())
		PyErr_Print();

	unsigned char *result_array = (unsigned char *)PyArray_DATA(python_result_array);

	if (PyErr_Occurred())
		PyErr_Print();

	Py_DECREF(numpyArray);
	Py_DECREF(numpyCutParam);

	return result_array;
}
