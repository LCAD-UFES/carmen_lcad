#include <Python.h>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <numeric>

using namespace std;

int main(int argc, char* argv[])
{
	PyObject *pName, *pModule, *pDict, *pFunc;
    PyObject *pArgs, *pValue;
    int i;

    cout << argv[0] << endl;
    cout << argv[1] << endl;
    cout << argv[2] << endl;
    
    //if (argc < 3) {
    //    fprintf(stderr,"Usage: call pythonfile funcname [args]\n");
    //    return 1;
    //}
    
    setenv("PYTHONPATH", ".", 1);
    Py_Initialize();
    //PySys_SetArgvEx(argc, argv, false);
    //PyRun_SimpleString("import sys\nprint(sys.argv)");
    pName = PyUnicode_FromString("demo");
    /* Error checking of pName left out */

    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, "main");
        /* pFunc is a new reference */

        if (pFunc && PyCallable_Check(pFunc)) {
            pArgs = PyTuple_New(argc - 3);
            for (i = 0; i < argc - 3; ++i) {
                pValue = PyLong_FromLong(atoi("foda-se"));
                if (!pValue) {
                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");
                    return 1;
                }
                /* pValue reference stolen here: */
                PyTuple_SetItem(pArgs, i, pValue);
            }
            pValue = PyObject_CallObject(pFunc, pArgs);
            cout << "0" << endl;
            //Py_DECREF(pArgs);
            if (pValue != NULL) {
                printf("Result of call: %ld\n", PyLong_AsLong(pValue));
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return 1;
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", "main");
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", "demo");
        return 1;
    }
    Py_Finalize();

    cout << "FIM!!!!!!!!!" << endl;
	return 0;
}
