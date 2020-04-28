#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <numpy/arrayobject.h>
#include <carmen/libsqueeze_seg.h>
#include <iostream>
#include <carmen/carmen.h>
#include <prob_map.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <stdlib.h> /* getenv */

PyObject *python_libsqueeze_seg_process_point_cloud_function;
PyObject *python_libsqueeze_seg_save_npy;

void initialize_python_path_squeezeseg()
{
	char* pyPath;
	char* pPath;
	char* squeezePath;
	pyPath = (char *) "PYTHONPATH=";
  	pPath = getenv ("CARMEN_HOME");
	squeezePath = (char *) "/sharedlib/libsqueeze_seg_v2/src";
    char * path = (char *) malloc(1 + strlen(pyPath) + strlen(pPath)+ strlen(squeezePath));
	strcpy(path, pyPath);
    strcat(path, pPath);
    strcat(path, squeezePath);
	putenv(path);
}

void
initialize_python_context()
{
	initialize_python_path_squeezeseg();
	
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

long long int*
libsqueeze_seg_process_point_cloud(int vertical_resolution, int shots_to_squeeze, double* point_cloud, double timestamp)
{
	//printf("libsqueeze_seg_process_point_cloud\n");
	npy_intp dims[3] = {vertical_resolution, shots_to_squeeze, 5};
	double time[1];
	time[0] = timestamp;
	npy_intp dimstamp[1] = {1};
	
	PyObject* numpyTimestamp = PyArray_SimpleNewFromData(1, dimstamp, NPY_DOUBLE, &time[0]);
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_DOUBLE, point_cloud);

	if (PyErr_Occurred())
		        PyErr_Print();

	PyArrayObject* python_result_array = (PyArrayObject*)PyObject_CallFunctionObjArgs(python_libsqueeze_seg_process_point_cloud_function, numpyArray, numpyTimestamp, NULL);

	if (PyErr_Occurred())
	        PyErr_Print();

	long long int *result_array = (long long int*)PyArray_DATA(python_result_array);

	if (PyErr_Occurred())
        PyErr_Print();

	Py_DECREF(numpyTimestamp);
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
fill_view_vector(double horizontal_angle, double vertical_angle, double range, double intensity, double* view, int line)
{
	if (range > 0 && range < 200) // this causes n_points to become wrong (needs later correction)
	{
		tf::Point point = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
		double x = round(point.x() * 100.0) / 100.0;
		double y = round(point.y() * 100.0) / 100.0;
		double z = round(point.z() * 100.0) / 100.0;
		intensity = round(intensity * 100.0) / 100.0;
		//double raiz_soma_quadrados = sqrt(x * x + y * y + z * z);
		view[line * 5] = x;
		view[(line * 5) + 1] = y;
		view[(line * 5) + 2] = z;
		view[(line * 5) + 3] = intensity;
		view[(line * 5) + 4] = range;
	} else {
		view[line * 5] = 0.0;
		view[(line * 5) + 1] = 0.0;
		view[(line * 5) + 2] = 0.0;
		view[(line * 5) + 3] = 0.0;
		view[(line * 5) + 4] = 0.0;
	}
}

long long int *
libsqueeze_seg_process_moving_obstacles_cells(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *sensors_params)
{
	long long int* squeezeseg_segmented;
    double timestamp = velodyne_message->timestamp;
    int shots_to_squeeze = velodyne_message->number_of_32_laser_shots;
    int vertical_resolution = sensors_params[sensor_number].vertical_resolution;
    int number_of_points = vertical_resolution * shots_to_squeeze;
    double view[number_of_points * 5];
    //printf("Shots: %d from timestamp %lf\n", velodyne_message->number_of_32_laser_shots, timestamp);
    for (int j = vertical_resolution, line = 0; j > 0; j--)
    {
        for (int i = 0; i < shots_to_squeeze; i++, line++)
        {
            double vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sensors_params[sensor_number].vertical_correction[j]));
            double horizontal_angle = carmen_normalize_theta(-carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
            double range = (((double)velodyne_message->partial_scan[i].distance[sensors_params[sensor_number].ray_order[j]]) / 500.0);
            double intensity = ((double)velodyne_message->partial_scan[i].intensity[sensors_params[sensor_number].ray_order[j]]) / 100.0;
            fill_view_vector(horizontal_angle, vertical_angle, range, intensity, &view[0], line);
        }
    }
	squeezeseg_segmented = libsqueeze_seg_process_point_cloud(vertical_resolution, shots_to_squeeze, &view[0], timestamp);
	return squeezeseg_segmented;
}


/*Functions for create dataset*/
void
initialize_python_dataset()
{
	initialize_python_path_squeezeseg();
	
	Py_Initialize();
	// import_array();

	PyObject *python_module_name = PyString_FromString((char *) "save_npy_dataset");

	PyObject *python_module = PyImport_Import(python_module_name);

	if (python_module == NULL)
	{
		Py_Finalize();
		exit(printf("Error: The python_module save_npy_dataset could not be loaded.\nMaybe PYTHONPATH is not set.\n"));
	}
	Py_DECREF(python_module_name);

	
	python_libsqueeze_seg_save_npy = PyObject_GetAttrString(python_module, (char *) "squeeze_seg_save_npy");

	if (python_libsqueeze_seg_save_npy == NULL || !PyCallable_Check(python_libsqueeze_seg_save_npy))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the squeeze_seg_save_npy.\n"));
	}
	
	printf("Success: Loaded squeeze_seg_save_npy\n");

}

void
libsqueeze_seg_save_npy_for_train(int vertical_resolution, int shots_to_squeeze, double* point_cloud, double timestamp)
{
	//printf("libsqueeze_seg_process_point_cloud\n");
	npy_intp dims[3] = {vertical_resolution, shots_to_squeeze, 6};
	double time[1];
	time[0] = timestamp;
	npy_intp dimstamp[1] = {1};
	
	PyObject* numpyTimestamp = PyArray_SimpleNewFromData(1, dimstamp, NPY_DOUBLE, &time[0]);
	PyObject* numpyArray = PyArray_SimpleNewFromData(3, dims, NPY_DOUBLE, point_cloud);

	if (PyErr_Occurred())
		        PyErr_Print();

	PyArrayObject* python_result_array = (PyArrayObject*)PyObject_CallFunctionObjArgs(python_libsqueeze_seg_save_npy, numpyArray, numpyTimestamp, NULL);

	if (PyErr_Occurred())
        PyErr_Print();


	Py_DECREF(numpyTimestamp);
	Py_DECREF(numpyArray);
	Py_DECREF(python_result_array);
	free(point_cloud);
}

void
fill_train_vector(double horizontal_angle, double vertical_angle, double range, double intensity, double* data_train, int line)
{
	if (range > 0 && range < 200) // this causes n_points to become wrong (needs later correction)
	{
		tf::Point point = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
		double x = round(point.x() * 100.0) / 100.0;
		double y = round(point.y() * 100.0) / 100.0;
		double z = round(point.z() * 100.0) / 100.0;
		intensity = round(intensity * 100.0) / 100.0;
		//double raiz_soma_quadrados = sqrt(x * x + y * y + z * z);
		data_train[line * 6] = x;
		data_train[(line * 6) + 1] = y;
		data_train[(line * 6) + 2] = z;
		data_train[(line * 6) + 3] = intensity;
		data_train[(line * 6) + 4] = range;
		data_train[(line * 6) + 5] = 0.0; //label
	} else {
		data_train[line * 6] = 0.0;
		data_train[(line * 6) + 1] = 0.0;
		data_train[(line * 6) + 2] = 0.0;
		data_train[(line * 6) + 3] = 0.0;
		data_train[(line * 6) + 4] = 0.0;
		data_train[(line * 6) + 5] = 0.0; //label
	}
}

void 
libsqueeze_seg_fill_label(int line, double label, double* data_train)
{
	data_train[line + 5] = label;
}

void
libsqueeze_seg_using_detections(vector<bbox_t> &predictions, vector<vector<image_cartesian>> &clustered_points, double* data_train, int vertical_resolution, int number_of_laser_shots)
{
	printf("libsqueeze_seg_using_detections\n");
	unsigned int cont = 0;
	vector<vector<image_cartesian>> classified;
	vector<image_cartesian> cluster;
	vector<int> moving_object_cluster_index;
	
	unsigned int predictions_size = predictions.size();
	for (unsigned int h = 0; h < predictions_size; h++)
	{
		unsigned int number_of_clusters = clustered_points.size();
		for (unsigned int i = 0; i < number_of_clusters; i++)
		{
			unsigned int cluster_size = clustered_points[i].size();
			//unsigned int min_points_inside_bbox = cluster_size / 5;
			bool is_moving_obstacle = false;
			unsigned int contCar = 0, contPerson = 0, contBycicle = 0, contTrain = 0;
			for (unsigned int j = 0; j < cluster_size; j++)
			{
				if ((unsigned int) clustered_points[i][j].image_x >=  predictions[h].x &&
					(unsigned int) clustered_points[i][j].image_x <= (predictions[h].x + predictions[h].w) &&
					(unsigned int) clustered_points[i][j].image_y >=  predictions[h].y &&
					(unsigned int) clustered_points[i][j].image_y <= (predictions[h].y + predictions[h].h))
				{
					switch (predictions[h].obj_id)
					{
						case 0: //person
							contPerson++;
							break;
						case 1: //bicycle
							contBycicle++;
							break;
						case 2: //car
							contCar++;
							break;
						case 3: //motorbike
							contBycicle++;
							break;
						case 5: //bus
							contCar++;
							break;
						case 6: //train
							contTrain++;
							break;
						case 7: //truck
							contCar++;
							break;
					}
					cont++;
					if (contCar > (cluster_size / 5) ||
						contPerson > (cluster_size / 5) ||
						contBycicle > (cluster_size / 5) ||
						contTrain > (cluster_size / 5))
					{
						is_moving_obstacle = true;
						break;
					}
				} // End if
			} // End for cluster
			if (is_moving_obstacle)
			{
				for (unsigned int j = 0; j < cluster_size; j++)
				{
					int squeeze_index = (int) (vertical_resolution - clustered_points[i][j].ray_number) * number_of_laser_shots * 6 + clustered_points[i][j].shot_number * 6 + 5;
					if (contCar > contPerson && contCar > contBycicle && contCar > contTrain)
					{
						libsqueeze_seg_fill_label(squeeze_index, 1.0, data_train);
					}
					else
					{
						if (contPerson > contBycicle && contPerson > contTrain)
						{
							libsqueeze_seg_fill_label(squeeze_index, 2.0, data_train);
						}
						else
						{
							if (contBycicle > contTrain)
							{
								libsqueeze_seg_fill_label(squeeze_index, 3.0, data_train);
							}
							else
							{
								libsqueeze_seg_fill_label(squeeze_index, 4.0, data_train);
							}
						}
					}
				}
			}// End Is moving obstacle
		}
	}
}
	
double *
libsqueeze_seg_data_for_train(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *sensors_params)
{
	int shots_to_squeeze = velodyne_message->number_of_32_laser_shots;
    int vertical_resolution = sensors_params[sensor_number].vertical_resolution;
    int number_of_points = vertical_resolution * shots_to_squeeze;
	//double data_train[number_of_points * 6];
	double * data_train = (double *) malloc (number_of_points * 6 * sizeof(double));

    for (int j = vertical_resolution, line = 0; j > 0; j--)
    {
        for (int i = 0; i < shots_to_squeeze; i++, line++)
        {
            double vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sensors_params[sensor_number].vertical_correction[j]));
            double horizontal_angle = carmen_normalize_theta(-carmen_degrees_to_radians(-velodyne_message->partial_scan[i].angle));
            double range = (((double)velodyne_message->partial_scan[i].distance[sensors_params[sensor_number].ray_order[j]]) / 500.0);
            double intensity = ((double)velodyne_message->partial_scan[i].intensity[sensors_params[sensor_number].ray_order[j]]) / 100.0;
            fill_train_vector(horizontal_angle, vertical_angle, range, intensity, data_train, line);
        }
    }
	return data_train;
}
