#include "libdeeplab.h"

#include <Python.h>
#include <numpy/arrayobject.h>


//////// Python
PyObject *python_semantic_segmentation_function;
PyObject *python_get_label_name_by_number_function;
PyObject *python_is_moving_object_function;


void
initialize_visualize_module()
{
	PyObject *python_module_name = PyString_FromString((char *) "visualize");

	PyObject *python_module = PyImport_Import(python_module_name);

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module visualize could not be loaded.\nMay be PYTHON_PATH is not set.\n"));
	}
	Py_DECREF(python_module_name);

	python_get_label_name_by_number_function = PyObject_GetAttrString(python_module, (char *) "get_label_name_by_number");

	if (python_get_label_name_by_number_function == NULL || !PyCallable_Check(python_get_label_name_by_number_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the python_get_label_name_by_number_function.\n"));
	}

	python_is_moving_object_function = PyObject_GetAttrString(python_module, (char *) "is_moving_object");

	if (python_is_moving_object_function == NULL || !PyCallable_Check(python_is_moving_object_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the python_is_moving_object_function.\n"));
	}
}


void
initialize_inference_context()
{
	Py_Initialize();
	import_array();

	PyObject *python_module_name = PyString_FromString((char *) "bridge");

	PyObject *python_module = PyImport_Import(python_module_name);

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module bridge could not be loaded.\nMay be PYTHON_PATH is not set.\n"));
	}
	Py_DECREF(python_module_name);

	python_semantic_segmentation_function = PyObject_GetAttrString(python_module, (char *) "process_image");

	if (python_semantic_segmentation_function == NULL || !PyCallable_Check(python_semantic_segmentation_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the python_semantic_segmentation_function.\n"));
	}

	initialize_visualize_module();
}



unsigned char*
process_image(int width, int height, unsigned char *image)
{
	//create shape for numpy array
	npy_intp dims[3] = {height, width, 3};
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image);

	PyObject* return_array;

	return_array = PyObject_CallFunction(python_semantic_segmentation_function, (char *) "(O)", numpyArray);

	unsigned char *z = (unsigned char*) PyArray_DATA(return_array);

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray);
	// Py_DECREF(return_array);

	return z;
}


unsigned char*
get_label_name_by_number(unsigned char label_number)
{
	PyObject* return_str = PyObject_CallFunction(python_get_label_name_by_number_function, (char *) "B", label_number);

	static unsigned char *label_name = NULL;
	if (label_name == NULL)
		label_name = (unsigned char *) malloc(256 * sizeof(unsigned char));

	PyArg_ParseTuple(return_str, "s", label_name);

	if (PyErr_Occurred())
        PyErr_Print();

	return label_name;
}


int
is_moving_object(unsigned char label_number)
{
	PyObject* return_int = PyObject_CallFunction(python_is_moving_object_function, (char *) "B", label_number);

	int result;
	PyArg_ParseTuple(return_int, "i", &result);

	if (PyErr_Occurred())
        PyErr_Print();

	return result;
}
