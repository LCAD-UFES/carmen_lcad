/*
 * libfcnpy.cpp
 *
 *  Created on: Nov 29, 2018
 *      Author: vinicius
 */
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION


#include <Python.h>
#include <numpy/arrayobject.h>
#include "lib_neural_mapper_py.h"
#include <opencv/cv.hpp>



//////// Python
PyObject *python_mapper_segmentation_function;

//Adaptar para as minhas funcoes de inferencia
void
initialize_inference_context_mapper_()
{
	Py_Initialize();
	printf("python neural mapper started \n");

	#define NUMPY_IMPORT_ARRAY_RETVAL

	import_array();

	PyObject *python_module_name = PyUnicode_FromString((char *) "neural_map_inference");

	PyObject *python_module = PyImport_Import(python_module_name);

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module neural_map_inference could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
	}

	python_mapper_segmentation_function = PyObject_GetAttrString(python_module, (char *) "process_image");

	if (python_mapper_segmentation_function == NULL || !PyCallable_Check(python_mapper_segmentation_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the python_mapper_segmentation_function.\n"));
	}
}


PyListObject *
convert_rddf_array(int size, double *a)
{
	PyObject *list = Py_BuildValue("[]");

	for (int i=0; i<size; i++)
	{
//		printf("%lf \n", a[i]);
		PyObject *element = Py_BuildValue("(d)", a[i]);
		PyList_Append(list, element);
		Py_DECREF(element);
	}
	return (PyListObject *)list;
}


double*
process_map_neural_mapper(int size, carmen_map_t *map_max, carmen_map_t *map_mean, carmen_map_t *map_min, carmen_map_t *map_numb, carmen_map_t *map_std)
{
	//create shape for numpy array
	int total_size = size*size;
	npy_intp dims[1] = {total_size};
	printf("Test size:%d  \n", total_size);

//	PyListObject *python_a1 = convert_rddf_array(size, image1);
//	PyListObject *python_a2 = convert_rddf_array(size, image2);
//	PyListObject *python_a3 = convert_rddf_array(size, image3);
//	PyListObject *python_a4 = convert_rddf_array(size, image4);
//	PyListObject *python_a5 = convert_rddf_array(size, image5);

	PyObject* numpyArray1 = PyArray_SimpleNewFromData(1, dims, NPY_DOUBLE, map_max->complete_map);
	PyObject* numpyArray2 = PyArray_SimpleNewFromData(1, dims, NPY_DOUBLE, map_mean->complete_map);
	PyObject* numpyArray3 = PyArray_SimpleNewFromData(1, dims, NPY_DOUBLE, map_min->complete_map);
	PyObject* numpyArray4 = PyArray_SimpleNewFromData(1, dims, NPY_DOUBLE, map_numb->complete_map);
	PyObject* numpyArray5 = PyArray_SimpleNewFromData(1, dims, NPY_DOUBLE, map_std->complete_map);
	//Um numpyArraypra cada

	PyArrayObject* return_array;
	return_array = (PyArrayObject*)PyObject_CallFunctionObjArgs(python_mapper_segmentation_function, numpyArray1, numpyArray2, numpyArray3, numpyArray4, numpyArray5, NULL);
//	return_array = PyObject_CallFunction(python_mapper_segmentation_function, (char *) "(OOOOO)", numpyArray1, numpyArray2, numpyArray3, numpyArray4, numpyArray5);

	if (PyErr_Occurred())
        PyErr_Print();

//	float *result_array = (float*)PyArray_DATA(python_result_array);
	double *z = (double*) PyArray_DATA(return_array);

	Py_DECREF(numpyArray1);
	Py_DECREF(numpyArray2);
	Py_DECREF(numpyArray3);
	Py_DECREF(numpyArray4);
	Py_DECREF(numpyArray5);
	Py_DECREF(return_array);

	return z;
}


