#include "libdeeplab.h"

#include <Python.h>
#include <numpy/arrayobject.h>


//////// Python
PyObject *python_semantic_segmentation_function;

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
		exit(printf("Error: The python_module could not be loaded.\nMay be PYTHON_PATH is not set.\n"));
	}
	Py_DECREF(python_module_name);

	python_semantic_segmentation_function = PyObject_GetAttrString(python_module, (char *) "process_image");

	if (python_semantic_segmentation_function == NULL || !PyCallable_Check(python_semantic_segmentation_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the python_semantic_segmentation_function.\n"));
	}
}



void
process_image(int width, int height, unsigned char *image)
{
	//create shape for numpy array
	npy_intp dims[3] = {height, width, 3};
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, image);

	PyObject_CallFunctionObjArgs(python_semantic_segmentation_function, numpyArray, NULL);

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray);
}