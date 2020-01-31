#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include "libsalsanet.h"
#include <numpy/arrayobject.h>
#include <carmen/carmen.h>
#include <prob_map.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>

#define NUMPY_IMPORT_ARRAY_RETVAL 

PyObject *python_libsalsanet_process_point_cloud_function;

void
initialize_python_context_salsanet()
{
	Py_Initialize();
	import_array();
	PyObject *python_module = PyImport_ImportModule("run_salsanet");

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module run_salsanet could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
	}

	PyObject *python_initialize_function = PyObject_GetAttrString(python_module, (char *) "initialize");

	if (python_initialize_function == NULL || !PyCallable_Check(python_initialize_function))
	{
		Py_Finalize();
		exit (printf("Error: Could not load the python_initialize_function.\n"));
	}
	PyObject *python_arguments = Py_BuildValue("(i)", 32);

	PyObject_CallObject(python_initialize_function, python_arguments);

	//Py_DECREF(python_arguments);
	//Py_DECREF(python_initialize_function);

	python_libsalsanet_process_point_cloud_function = PyObject_GetAttrString(python_module, (char *) "salsanet_process_point_cloud");

	if (python_libsalsanet_process_point_cloud_function == NULL || !PyCallable_Check(python_libsalsanet_process_point_cloud_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the salsanet_process_point_cloud.\n"));
	}
	
	printf("Success: Loaded SalsaNet\n");

}

long long int*
libsalsanet_process_point_cloud(int vertical_resolution, int shots_to_squeeze, double* point_cloud, double timestamp)
{
	printf("libsalsanet_process_point_cloud\n");
	npy_intp dims[3] = {vertical_resolution, shots_to_squeeze, 5};
	double time[1];
	time[0] = timestamp;
	npy_intp dimstamp[1] = {1};
	
	PyObject* numpyTimestamp = PyArray_SimpleNewFromData(1, dimstamp, NPY_DOUBLE, &time[0]);
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_DOUBLE, point_cloud);

	if (PyErr_Occurred())
		        PyErr_Print();

	PyArrayObject* python_result_array = (PyArrayObject*)PyObject_CallFunctionObjArgs(python_libsalsanet_process_point_cloud_function, numpyArray, numpyTimestamp, NULL);

	if (PyErr_Occurred())
	        PyErr_Print();

	long long int *result_array = (long long int*)PyArray_DATA(python_result_array);

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyArray);
	//Py_DECREF(python_result_array);

	return result_array;
}

inline double round(double val)
{
    if (val < 0)
        return ceil(val - 0.5);
    return floor(val + 0.5);
}

void
fill_data_vector(double horizontal_angle, double vertical_angle, double range, double intensity, double* data, int line)
{
	if (range > 0 && range < 200) // this causes n_points to become wrong (needs later correction)
	{
		tf::Point point = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
		double x = round(point.x() * 100.0) / 100.0;
		double y = round(point.y() * 100.0) / 100.0;
		double z = round(point.z() * 100.0) / 100.0;
		intensity = round(intensity * 100.0) / 100.0;
		data[line * 5] = x;
		data[(line * 5) + 1] = y;
		data[(line * 5) + 2] = z;
		data[(line * 5) + 3] = intensity;
		data[(line * 5) + 4] = range;
	} else {
		data[line * 5] = 0.0;
		data[(line * 5) + 1] = 0.0;
		data[(line * 5) + 2] = 0.0;
		data[(line * 5) + 3] = 0.0;
		data[(line * 5) + 4] = 0.0;
	}
}

long long int *
erase_moving_obstacles_cells_salsanet(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *sensors_params)
{
	long long int *salsanet_segmented = NULL;
	double timestamp = velodyne_message->timestamp;
	int shots_to_squeeze = velodyne_message->number_of_32_laser_shots;
	int vertical_resolution = sensors_params[sensor_number].vertical_resolution;
	int number_of_points = vertical_resolution * shots_to_squeeze;
	double data[number_of_points * 5];
	printf("Salsanet: %d from timestamp %lf\n", velodyne_message->number_of_32_laser_shots, timestamp);
	for (int j = 0, line = 0; j < vertical_resolution; j++)
	{
		for (int i = 0; i < shots_to_squeeze; i++, line++)
		{
			double vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sensors_params[sensor_number].vertical_correction[j]));
			double horizontal_angle = carmen_normalize_theta(carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
			double range = (((double)velodyne_message->partial_scan[i].distance[sensors_params[sensor_number].ray_order[j]]) / 500.0);
			double intensity = ((double)velodyne_message->partial_scan[i].intensity[sensors_params[sensor_number].ray_order[j]]) / 100.0;
			fill_data_vector(horizontal_angle, vertical_angle, range, intensity, &data[0], line);
		}
	}
	salsanet_segmented = libsalsanet_process_point_cloud(vertical_resolution, shots_to_squeeze, &data[0], timestamp);
	return salsanet_segmented;
}


