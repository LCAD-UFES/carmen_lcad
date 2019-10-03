#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
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
		exit(printf("Error: The python_module run_squeeze_seg could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
	}
	Py_DECREF(python_module_name);

	python_libsqueeze_seg_process_point_cloud_function = PyObject_GetAttrString(python_module, (char *) "squeeze_seg_process_point_cloud");

	if (python_libsqueeze_seg_process_point_cloud_function == NULL || !PyCallable_Check(python_libsqueeze_seg_process_point_cloud_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the squeeze_seg_process_point_cloud.\n"));
	}
	printf("Success: Loaded SqueezeSeg\n");

}



float*
libsqueeze_seg_process_point_cloud(int vertical_resolution, int shots_to_squeeze, float* point_cloud, double timestamp)
{
	//printf("SqueezeSeg process point was called\n");
	npy_intp dims[3] = {vertical_resolution, shots_to_squeeze, 5};
	double time[1];
	time[0] = timestamp;
	npy_intp dimstamp[1] = {1};
	//	printf("Dims Mounted, Number of points %d\n", number_of_points);
//	for (unsigned int i = 0; i < number_of_points; i++){
//		for (unsigned int j = 0; j < 5; j++)
//			printf("%.2f\t", point_cloud[(i * 5) + j]);
//		printf("\n");
//	}
	PyObject* numpyTimestamp = PyArray_SimpleNewFromData(1, dimstamp, NPY_DOUBLE, &time[0]);
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_FLOAT, point_cloud);
	printf("numpyArray Mounted\n");
	//PyArrayObject* python_result_array = (PyArrayObject*) PyObject_CallFunction(python_libsqueeze_seg_process_point_cloud_function, (char *) "(O)", numpyArray, numpyTimestamp);
	PyArrayObject* python_result_array = (PyArrayObject*)PyObject_CallFunctionObjArgs(python_libsqueeze_seg_process_point_cloud_function, numpyArray, numpyTimestamp, NULL);

	float *result_array = (float*)PyArray_DATA(python_result_array);

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray);
	Py_DECREF(python_result_array);
	//exit(0);

	return result_array;
}


void
libsqueeze_seg_test()
{
	printf("SqueezeSeg included\n");
}
