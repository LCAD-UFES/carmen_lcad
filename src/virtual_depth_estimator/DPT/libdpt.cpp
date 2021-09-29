#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include "libdpt.h"
#include <numpy/arrayobject.h>
#include <stdlib.h> /* getenv */

#define NUMPY_IMPORT_ARRAY_RETVAL

PyObject *python_libdpt_process_image_function;

void initialize_python_path_dpt()
{
	char* pyPath;
	char* pPath;
	char* dptPath;
	pyPath = (char *) "PYTHONPATH=";
  	pPath = getenv ("CARMEN_HOME");
	dptPath = (char *) "/src/virtual_depth_estimator/DPT";
    char * path = (char *) malloc(1 + strlen(pyPath) + strlen(pPath)+ strlen(dptPath));
	strcpy(path, pyPath);
    strcat(path, pPath);
    strcat(path, dptPath);
	putenv(path);
}

void
initialize_python_context()
{
	initialize_python_path_dpt();
	Py_Initialize();
	import_array();
	PyObject *python_module = PyImport_ImportModule("run_dpt");
	
	if (PyErr_Occurred())
		        PyErr_Print();

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module run_dpt could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
	}

	if (PyErr_Occurred())
		        PyErr_Print();

	PyObject *python_initialize_function = PyObject_GetAttrString(python_module, (char *) "initialize");

	if (PyErr_Occurred())
		        PyErr_Print();
	if (python_initialize_function == NULL || !PyCallable_Check(python_initialize_function))
	{
		Py_Finalize();
		exit (printf("Error: Could not load the python_initialize_function.\n"));
	}
	PyObject *python_arguments = Py_BuildValue("()");

	if (PyErr_Occurred())
		        PyErr_Print();

	PyObject_CallObject(python_initialize_function, python_arguments);

	if (PyErr_Occurred())
		        PyErr_Print();

	// Py_DECREF(python_arguments);
	// Py_DECREF(python_initialize_function);

	python_libdpt_process_image_function = PyObject_GetAttrString(python_module, (char *) "dpt_process_image");
	
	if (PyErr_Occurred())
		        PyErr_Print();

	if (python_libdpt_process_image_function == NULL || !PyCallable_Check(python_libdpt_process_image_function))
	{
		// Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the dpt_process_image.\n"));
	}

	if (PyErr_Occurred())
		        PyErr_Print();

	printf("Success: Loaded DPT\n");

}

unsigned char*
libdpt_process_image(int width, int height, unsigned char *image, double timestamp)
{
	// printf("libdpt_process_image\n");
	//create shape for numpy array
	npy_intp dims[3] = {height, width, 3};
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image);
	double time[1];
	time[0] = timestamp;
	npy_intp dimstamp[1] = {1};
	
	PyObject* numpyTimestamp = PyArray_SimpleNewFromData(1, dimstamp, NPY_DOUBLE, &time[0]);

	if (PyErr_Occurred())
		        PyErr_Print();
	PyArrayObject* python_result_array = (PyArrayObject*) PyObject_CallFunctionObjArgs(python_libdpt_process_image_function, numpyArray, numpyTimestamp, NULL);

	if (PyErr_Occurred())
	        PyErr_Print();

	unsigned char*result_array = (unsigned char*)PyArray_DATA(python_result_array);

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray);
	Py_DECREF(numpyTimestamp);
	//Py_DECREF(python_result_array);

	return result_array;
	// return 1;
}

