#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <numpy/arrayobject.h>
#include "libsqueeze_seg.h"
#include <iostream>


PyObject *python_libsqueeze_seg_process_point_cloud_function;

void
initialize_python_context()
{
	Py_Initialize();
	import_array();

	PyObject *python_module_name = PyString_FromString((char *) "run_squeeze_seg");

	PyObject *python_module = PyImport_Import(python_module_name);

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module run_squeeze_seg could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
	}
	Py_DECREF(python_module_name);

	PyObject *python_initialize_function = PyObject_GetAttrString(python_module, (char *) "initialize");

	if (python_initialize_function == NULL || !PyCallable_Check(python_initialize_function))
	{
		Py_Finalize();
		exit (printf("Error: Could not load the python_initialize_function.\n"));
	}
	PyObject *python_arguments = Py_BuildValue("(ii)", 32, 1024);

	PyObject_CallObject(python_initialize_function, python_arguments);

	Py_DECREF(python_arguments);
	Py_DECREF(python_initialize_function);

	python_libsqueeze_seg_process_point_cloud_function = PyObject_GetAttrString(python_module, (char *) "squeeze_seg_process_point_cloud");

	if (python_libsqueeze_seg_process_point_cloud_function == NULL || !PyCallable_Check(python_libsqueeze_seg_process_point_cloud_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the squeeze_seg_process_point_cloud.\n"));
	}
	
	printf("Success: Loaded SqueezeSeg\n");

}

int*
libsqueeze_seg_process_point_cloud(int vertical_resolution, int shots_to_squeeze, float* point_cloud, double timestamp)
{
	printf("libsqueeze_seg_process_point_cloud\n");
	npy_intp dims[3] = {vertical_resolution, shots_to_squeeze, 5};
	double time[1];
	time[0] = timestamp;
	npy_intp dimstamp[1] = {1};
	
	PyObject* numpyTimestamp = PyArray_SimpleNewFromData(1, dimstamp, NPY_DOUBLE, &time[0]);
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_FLOAT, point_cloud);
	//PyArrayObject* python_result_array = (PyArrayObject*) PyObject_CallFunction(python_libsqueeze_seg_process_point_cloud_function, (char *) "(O)", numpyArray, numpyTimestamp);
    //auto t1 = std::chrono::high_resolution_clock::now();
	PyArrayObject* python_result_array = (PyArrayObject*)PyObject_CallFunctionObjArgs(python_libsqueeze_seg_process_point_cloud_function, numpyArray, numpyTimestamp, NULL);
	if (PyErr_Occurred())
	        PyErr_Print();
	//auto t2 = std::chrono::high_resolution_clock::now();
	//auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    //std::cout << "Inference Duration: " << duration << "ms" << std::endl;
	
	int *result_array = (int*)PyArray_DATA(python_result_array);

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray);
	Py_DECREF(python_result_array);

	return result_array;
}

