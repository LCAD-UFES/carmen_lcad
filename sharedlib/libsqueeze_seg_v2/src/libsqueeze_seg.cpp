#include <Python.h>
#include <numpy/arrayobject.h>
#include "libsqueeze_seg.h"


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
		exit(printf("Error: The python_module bridge could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
	}
	Py_DECREF(python_module_name);

	python_libsqueeze_seg_process_point_cloud_function = PyObject_GetAttrString(python_module, (char *) "squeeze_seg_process_point_cloud");

	if (python_libsqueeze_seg_process_point_cloud_function == NULL || !PyCallable_Check(python_libsqueeze_seg_process_point_cloud_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the python_semantic_segmentation_function.\n"));
	}
}



float*
libsqueeze_seg_process_point_cloud(float * point_cloud, int shots_number)
{
	npy_intp dims[2] = {shots_number, 5};
	PyObject* numpyArray = PyArray_SimpleNewFromData(2, dims, NPY_FLOAT32, point_cloud);

	PyObject* python_result_array;

	python_result_array = PyObject_CallFunction(python_libsqueeze_seg_process_point_cloud_function, (char *) "(O)", numpyArray);

	float *result_array = (float*) PyArray_DATA(python_result_array);

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray);
	// Py_DECREF(return_array);

	return result_array;
}


void
libsqueeze_seg_test()
{
	printf("SqueezeSeg included\n");
}
