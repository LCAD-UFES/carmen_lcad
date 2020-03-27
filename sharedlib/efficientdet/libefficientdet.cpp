#include <Python.h>
#include "libefficientdet.h"
#include <numpy/arrayobject.h>
#include <stdlib.h> /* getenv */
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

// #include <iostream>
#define NUMPY_IMPORT_ARRAY_RETVAL


PyObject *python_libefficientdet_process_image_function;

void initialize_python_path_efficientdet()
{
	char* pyPath;
	char* pPath;
	char* effPath;
	pyPath = (char *) "PYTHONPATH=";
  	pPath = getenv ("CARMEN_HOME");
	effPath = (char *) "/sharedlib/efficientdet";
    char * path = (char *) malloc(1 + strlen(pyPath) + strlen(pPath)+ strlen(effPath));
	strcpy(path, pyPath);
    strcat(path, pPath);
    strcat(path, effPath);
	putenv(path);
}

void
initialize_Efficientdet()
{
	initialize_python_path_efficientdet();
	Py_Initialize();
	import_array();
	PyObject *python_module = PyImport_ImportModule("run_efficientdet");
	
	if (PyErr_Occurred())
		        PyErr_Print();

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module run_efficientdet could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
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
	//PyObject *python_arguments = Py_BuildValue("(ii)", width, height);

	if (PyErr_Occurred())
		        PyErr_Print();

	PyObject_CallObject(python_initialize_function, NULL);

	if (PyErr_Occurred())
		        PyErr_Print();

	// Py_DECREF(python_arguments);
	// Py_DECREF(python_initialize_function);

	python_libefficientdet_process_image_function = PyObject_GetAttrString(python_module, (char *) "efficientdet_process_image");
	
	if (PyErr_Occurred())
		        PyErr_Print();

	if (python_libefficientdet_process_image_function == NULL || !PyCallable_Check(python_libefficientdet_process_image_function))
	{
		// Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the efficientdet_process_image.\n"));
	}

	if (PyErr_Occurred())
		        PyErr_Print();

	printf("Success: Loaded efficientdet\n");

}

std::vector<bbox_t>
run_EfficientDet(unsigned char *image, int width, int height)
{
	printf("libefficientdet_process_image\n");
	//create shape for numpy array
	npy_intp dims[3] = {height, width, 3};
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image);

	/*double time[1];
	time[0] = timestamp;
	npy_intp dimstamp[1] = {1};
	
	PyObject* numpyTimestamp = PyArray_SimpleNewFromData(1, dimstamp, NPY_DOUBLE, &time[0]);
	*/

	if (PyErr_Occurred())
		        PyErr_Print();

	PyArrayObject* python_result_array = (PyArrayObject*)PyObject_CallFunctionObjArgs(python_libefficientdet_process_image_function, numpyArray, NULL);

	if (PyErr_Occurred())
	        PyErr_Print();

	int num_objs = (int)python_result_array->dimensions[0];
	double* result_array = (double*) PyArray_DATA(python_result_array);
	std::vector<bbox_t> bbox_vector;
	for(int i = 0; i < num_objs; i++)
	{
		bbox_t pred = {};
		pred.x = (unsigned int)		result_array[(i * 6)];
		pred.y = (unsigned int)		result_array[(i * 6) + 1];
		pred.w = (unsigned int)		result_array[(i * 6) + 2];
		pred.h = (unsigned int)		result_array[(i * 6) + 3];
		pred.prob = (float)			result_array[(i * 6) + 4];
		pred.obj_id = (unsigned int) result_array[(i * 6) + 5];
		pred.track_id = 0;
		if (pred.prob >= 0.2)
			bbox_vector.push_back(pred);
	}

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray);
	Py_DECREF(python_result_array);

	return bbox_vector;
}

