#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include "libadabins.h"
#include <numpy/arrayobject.h>
#include <stdlib.h> /* getenv */

#define NUMPY_IMPORT_ARRAY_RETVAL

PyObject *python_libadabins_process_image_function;

void initialize_python_path_adabins()
{
	char* pyPath;
	char* pPath;
	char* adabinsPath;
	pyPath = (char *) "PYTHONPATH=";
  	pPath = getenv ("CARMEN_HOME");
	adabinsPath = (char *) "/src/deep_mapper/Adabins";
    char * path = (char *) malloc(1 + strlen(pyPath) + strlen(pPath)+ strlen(adabinsPath));
	strcpy(path, pyPath);
    strcat(path, pPath);
    strcat(path, adabinsPath);
	putenv(path);
}

void
initialize_python_context()
{
	initialize_python_path_adabins();
	Py_Initialize();
	import_array();
	PyObject *python_module = PyImport_ImportModule("deep_mapper");
	
	if (PyErr_Occurred())
		        PyErr_Print();

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module deep_mapper could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
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

	python_libadabins_process_image_function = PyObject_GetAttrString(python_module, (char *) "inferenceDepth");
	
	if (PyErr_Occurred())
		        PyErr_Print();

	if (python_libadabins_process_image_function == NULL || !PyCallable_Check(python_libadabins_process_image_function))
	{
		// Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the inferenceDepth.\n"));
	}

	if (PyErr_Occurred())
		        PyErr_Print();

	printf("Success: Loaded Adabins\n");

}

unsigned char*
libadabins_process_image(int width, int height, unsigned char *image, int cut_param)
{
	// printf("libadabins_process_image\n");
	//create shape for numpy array
	npy_intp dims[3] = {height, width, 3};
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image);

	int time[1];
	time[0] = cut_param;
	npy_intp dimcut[1] = {1};
	
	PyObject* numpyCutParam = PyArray_SimpleNewFromData(1, dimcut, NPY_INT, &time[0]);
	
	if (PyErr_Occurred())
		        PyErr_Print();
	PyArrayObject* python_result_array = (PyArrayObject*) PyObject_CallFunctionObjArgs(python_libadabins_process_image_function, numpyArray, numpyCutParam, NULL);
	//PyArrayObject* python_result_array = (PyArrayObject*) PyObject_CallFunctionObjArgs(python_libadabins_process_image_function, numpyArray, NULL);

	if (PyErr_Occurred())
	        PyErr_Print();

	unsigned char*result_array = (unsigned char*)PyArray_DATA(python_result_array);


	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray);

	return result_array;
}

