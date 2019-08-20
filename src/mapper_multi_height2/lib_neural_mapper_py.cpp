/*
 * libfcnpy.cpp
 *
 *  Created on: Nov 29, 2018
 *      Author: vinicius
 */
#include "lib_neural_mapper_py.h"

#include <Python.h>
#include <numpy/arrayobject.h>
#include <opencv/cv.hpp>



//////// Python
PyObject *python_mapper_segmentation_function;

//Adaptar para as minhas funcoes de inferencia
void
initialize_inference_context_mapper()
{
	Py_Initialize();

	#define NUMPY_IMPORT_ARRAY_RETVAL

	import_array();

	PyObject *python_module_name = PyUnicode_FromString((char *) "neural_map_inference");

	PyObject *python_module = PyImport_Import(python_module_name);

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module neural_map_inference could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
	}
	Py_DECREF(python_module_name);

	python_mapper_segmentation_function = PyObject_GetAttrString(python_module, (char *) "process_image");

	if (python_mapper_segmentation_function == NULL || !PyCallable_Check(python_mapper_segmentation_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the python_mapper_segmentation_function.\n"));
	}

}


unsigned char*
process_image_nm(int width, int height, unsigned char *image1, unsigned char *image2, unsigned char *image3, unsigned char *image4, unsigned char *image5)
{
	//create shape for numpy array
	npy_intp dims[3] = {height, width, 3};

	PyObject* numpyArray1 = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image1); //NPY_DOUBLE, image1);
	PyObject* numpyArray2 = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image2); //NPY_DOUBLE, image2);
	PyObject* numpyArray3 = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image3); //NPY_DOUBLE, image3);
	PyObject* numpyArray4 = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image4); //NPY_DOUBLE, image4);
	PyObject* numpyArray5 = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image5); //NPY_DOUBLE, image5);
	//Um numpyArraypra cada

	PyObject* return_array;

	return_array = PyObject_CallFunction(python_mapper_segmentation_function, (char *) "(OOOOO)", numpyArray1, numpyArray2, numpyArray3, numpyArray4, numpyArray5);


	unsigned char *z = (unsigned char*) PyArray_DATA(return_array);

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray1);
	Py_DECREF(numpyArray2);
	Py_DECREF(numpyArray3);
	Py_DECREF(numpyArray4);
	Py_DECREF(numpyArray5);
	// Py_DECREF(return_array);

	return z;
}


