
/* File : pyCarmen.i */

%module(directors="1") pyCarmen
%feature("director:except") {
    if( $error != NULL ) {
        PyObject *ptype, *pvalue, *ptraceback;
        PyErr_Fetch( &ptype, &pvalue, &ptraceback );
        PyErr_Restore( ptype, pvalue, ptraceback );
        PyErr_Print();
        Py_Exit(1);
    }
} 
%{
#include "pyCarmenMessages.h"

%}
//#include "linemapping_messages.h"

//%typemap(memberin) float * {
 // int i;
 //int size = PyList_Size($input);
//
//  float $1[size];
//  	
//  for (i = 0; i < size; i++) {
//      $1[i] = $input[i];
//  }
//}



//// Map a Python sequence into any sized C double array
%typemap(in) double* {
  int i, my_len;
  if (!PySequence_Check($input)) {
      PyErr_SetString(PyExc_TypeError,"Expecting a sequence");
      return NULL;
  }

  my_len = PyObject_Length($input);
  double *temp = (double*)calloc(my_len,sizeof(double));
  //carmen_test_alloc(temp);
  for (i =0; i < my_len; i++) {
      PyObject *o = PySequence_GetItem($input,i);
      if (!PyFloat_Check(o) && !PyInt_Check(o)) {
         PyErr_SetString(PyExc_ValueError,"Expecting a sequence of doubles");
         return NULL;
      }
      temp[i] = PyFloat_AsDouble(o);
  }
  $1 = temp;
}

//// Map a Python sequence into any sized C int array
%typemap(in) int* {
  int i, my_len;
  if (!PySequence_Check($input)) {
      PyErr_SetString(PyExc_TypeError,"Expecting a sequence");
      return NULL;
  }

  my_len = PyObject_Length($input);
  int *temp = (int*)calloc(my_len,sizeof(int));
  //carmen_test_alloc(temp);
  for (i =0; i < my_len; i++) {
      PyObject *o = PySequence_GetItem($input,i);
      if (!PyFloat_Check(o) && !PyInt_Check(o)) {
         PyErr_SetString(PyExc_ValueError,"Expecting a sequence of doubles");
         return NULL;
      }
      temp[i] = (int)PyFloat_AsDouble(o);
  }
  $1 = temp;
}




//// Map a Python sequence into any sized C double array
%typemap(in) float* {
  int i, my_len;
  if (!PySequence_Check($input)) {
      PyErr_SetString(PyExc_TypeError,"Expecting a sequence");
      return NULL;
  }

  my_len = PyObject_Length($input);
  float *temp = (float*)calloc(my_len,sizeof(float));
  //carmen_test_alloc(temp);
  for (i =0; i < my_len; i++) {
      PyObject *o = PySequence_GetItem($input,i);
      if (!PyFloat_Check(o) && !PyInt_Check(o)) {
         PyErr_SetString(PyExc_ValueError,"Expecting a sequence of doubles");
         return NULL;
      }
      temp[i] = (float)PyFloat_AsDouble(o);
  }
  $1 = temp;
}




/*%typemap(out) carmen_linemapping_segment_set_t{

	PyObject* segs_start = PyList_New(0);
	PyObject* segs_end  = PyList_New(0);
	PyObject* segs_weights  = PyList_New(0);	
	
        int i;
	for(i=0; i<$1.num_segs; i++){
	     PyObject* segs_start_pt = PyList_New(0);
	     PyObject* pt_x = PyFloat_FromDouble($1.segs[i].p1.x);	
	     PyObject* pt_y = PyFloat_FromDouble($1.segs[i].p1.y);
	     PyList_Append(segs_start_pt, pt_x);
	     PyList_Append(segs_start_pt, pt_y);
	     PyList_Append(segs_start, segs_start_pt);

	     PyObject* segs_end_pt  = PyList_New(0);
	     PyObject* pt_x2 = PyFloat_FromDouble($1.segs[i].p2.x);
	     PyObject* pt_y2 = PyFloat_FromDouble($1.segs[i].p2.y);
	     PyList_Append(segs_end_pt, pt_x2);
	     PyList_Append(segs_end_pt, pt_y2);
	     PyList_Append(segs_end, segs_end_pt);

	     PyObject* weight = PyFloat_FromDouble($1.segs[i].weight);	
	     PyList_Append(segs_weights, weight);

   	     Py_DECREF(pt_x);
   	     Py_DECREF(pt_y);
   	     Py_DECREF(pt_x2);	
   	     Py_DECREF(pt_y2);
   	     Py_DECREF(weight);
   	     Py_DECREF(segs_start_pt);
   	     Py_DECREF(segs_end_pt);
	 }

	PyObject* retObj = PyDict_New();
	PyDict_SetItemString(retObj, "start_points", segs_start);
	PyDict_SetItemString(retObj, "end_points", segs_end);
	PyDict_SetItemString(retObj, "weights", segs_weights);

	Py_DECREF(segs_start);
	Py_DECREF(segs_end);
	Py_DECREF(segs_weights);	
	$result = retObj;
}*/



/*%typemap(out) carmen_map_t*{

	PyObject* map= PyList_New(0);
        int i,j;
	for(i=0; i<$1->config.x_size; i++){
	  PyObject* row = PyList_New(0);			

	  for(j=0; j<$1->config.y_size; j++){
		  PyObject* pt = PyFloat_FromDouble((double)$1->map[i][j]);	
		  PyList_Append(row, pt);
		  Py_DECREF(pt);
	  }

	  PyList_Append(map, row);
	  Py_DECREF(row);
	}

	PyObject* retObj = PyDict_New();
	PyDict_SetItemString(retObj, "map", map);
	PyDict_SetItemString(retObj, "resolution", PyFloat_FromDouble((double)$1->config.resolution));

	Py_DECREF(map);
	$result = retObj;
}*/


%typemap(out) carmen_laser_laser_message*{

	PyObject* laser_config = PyDict_New();
	PyObject* start_angle = PyFloat_FromDouble((double)$1->config.start_angle);
	PyObject* fov = PyFloat_FromDouble((double)$1->config.fov);
	PyObject* ar = PyFloat_FromDouble((double)$1->config.angular_resolution);
	PyObject* maximum_range = PyFloat_FromDouble((double)$1->config.maximum_range);
	PyObject* accuracy = PyFloat_FromDouble((double)$1->config.accuracy);
	//	PyObject* laser_type = PyFloat_FromDouble((double)$1->config.timestamp);
	PyDict_SetItemString(laser_config, "start_angle", start_angle);
	PyDict_SetItemString(laser_config, "fov", fov);
	PyDict_SetItemString(laser_config, "angular_resolution", ar);
	PyDict_SetItemString(laser_config, "maximum_range", maximum_range);	
	PyDict_SetItemString(laser_config, "accuracy", accuracy);	
	Py_DECREF(start_angle);
	Py_DECREF(fov);
	Py_DECREF(ar);
	Py_DECREF(maximum_range);
	Py_DECREF(accuracy);


	PyObject* range = PyList_New(0);
        int i;
	for(i=0; i<$1->num_readings; i++){
	  PyObject* pt = PyFloat_FromDouble((double)$1->range[i]);
	  PyList_Append(range, pt);
	  Py_DECREF(pt);
	}

	PyObject* retObj = PyDict_New();
	PyDict_SetItemString(retObj, "range", range);

	PyObject* ts = PyFloat_FromDouble((double)$1->timestamp);
	PyDict_SetItemString(retObj, "timestamp", ts);

	PyDict_SetItemString(retObj, "laser_config", laser_config);

	Py_DECREF(range);
	Py_DECREF(laser_config);
	Py_DECREF(ts);
	$result = retObj;
}



%typemap(out) carmen_robot_laser_message*{
	PyObject* laser_pose = PyList_New(0);
	PyObject* x = PyFloat_FromDouble((double)$1->laser_pose.x);
	PyObject* y = PyFloat_FromDouble((double)$1->laser_pose.y);
	PyObject* theta = PyFloat_FromDouble((double)$1->laser_pose.theta);
	PyList_Append(laser_pose, x);
	PyList_Append(laser_pose, y);
	PyList_Append(laser_pose, theta);
	Py_DECREF(x);
	Py_DECREF(y);
	Py_DECREF(theta);
	PyObject* robot_pose = PyList_New(0);
	PyObject* x2 = PyFloat_FromDouble((double)$1->robot_pose.x);
	PyObject* y2 = PyFloat_FromDouble((double)$1->robot_pose.y);
	PyObject* theta2 = PyFloat_FromDouble((double)$1->robot_pose.theta);
	PyList_Append(robot_pose, x2);
	PyList_Append(robot_pose, y2);
	PyList_Append(robot_pose, theta2);
	Py_DECREF(x2);
	Py_DECREF(y2);
	Py_DECREF(theta2);


  /** The laser message of the laser module (rawlaser) **/

	PyObject* laser_config = PyDict_New();
	PyObject* start_angle = PyFloat_FromDouble((double)$1->config.start_angle);
	PyObject* fov = PyFloat_FromDouble((double)$1->config.fov);
	PyObject* ar = PyFloat_FromDouble((double)$1->config.angular_resolution);
	PyObject* maximum_range = PyFloat_FromDouble((double)$1->config.maximum_range);
	PyObject* accuracy = PyFloat_FromDouble((double)$1->config.accuracy);
	//	PyObject* laser_type = PyFloat_FromDouble((double)$1->config.timestamp);
	PyDict_SetItemString(laser_config, "start_angle", start_angle);
	PyDict_SetItemString(laser_config, "fov", fov);
	PyDict_SetItemString(laser_config, "angular_resolution", ar);
	PyDict_SetItemString(laser_config, "maximum_range", maximum_range);	
	PyDict_SetItemString(laser_config, "accuracy", accuracy);	
	Py_DECREF(start_angle);
	Py_DECREF(fov);
	Py_DECREF(ar);
	Py_DECREF(maximum_range);
	Py_DECREF(accuracy);


	PyObject* range = PyList_New(0);
        int i;
	for(i=0; i<$1->num_readings; i++){
	  PyObject* pt = PyFloat_FromDouble((double)$1->range[i]);
	  PyList_Append(range, pt);
	  Py_DECREF(pt);
	}

	PyObject* retObj = PyDict_New();
	PyDict_SetItemString(retObj, "range", range);

	PyObject* ts = PyFloat_FromDouble((double)$1->timestamp);
	PyObject* tv = PyFloat_FromDouble((double)$1->tv);
	PyObject* rv = PyFloat_FromDouble((double)$1->rv);
	PyDict_SetItemString(retObj, "tv", tv);
	PyDict_SetItemString(retObj, "rv", rv);
	PyDict_SetItemString(retObj, "timestamp", ts);
	PyDict_SetItemString(retObj, "robot_pose", robot_pose);	
	PyDict_SetItemString(retObj, "laser_pose", laser_pose);	
	PyDict_SetItemString(retObj, "laser_config", laser_config);	

	Py_DECREF(robot_pose);
	Py_DECREF(laser_pose);
	Py_DECREF(range);
	Py_DECREF(ts);
	Py_DECREF(tv);
	Py_DECREF(rv);
	Py_DECREF(laser_config);
	$result = retObj;
}


%typemap(out) carmen_simulator_truepos_message*{

	PyObject* truepose = PyList_New(0);
	PyObject* x = PyFloat_FromDouble((double)$1->truepose.x);
	PyObject* y = PyFloat_FromDouble((double)$1->truepose.y);
	PyObject* theta = PyFloat_FromDouble((double)$1->truepose.theta);
	PyList_Append(truepose, x);
	PyList_Append(truepose, y);
	PyList_Append(truepose, theta);
	Py_DECREF(x);
	Py_DECREF(y);
	Py_DECREF(theta);

	
	PyObject* odometrypose = PyList_New(0);
	PyObject* x2 = PyFloat_FromDouble((double)$1->odometrypose.x);
	PyObject* y2 = PyFloat_FromDouble((double)$1->odometrypose.y);
	PyObject* theta2 = PyFloat_FromDouble((double)$1->odometrypose.theta);
	PyList_Append(odometrypose, x2);
	PyList_Append(odometrypose, y2);
	PyList_Append(odometrypose, theta2);
	Py_DECREF(x2);
	Py_DECREF(y2);
	Py_DECREF(theta2);

	PyObject* timestamp = PyFloat_FromDouble((double)$1->timestamp);
	
	//create the dictionary
	PyObject* sim_truepos = PyDict_New();
	PyDict_SetItemString(sim_truepos, "truepose", truepose);
	PyDict_SetItemString(sim_truepos, "odometrypose", odometrypose);
	PyDict_SetItemString(sim_truepos, "timestamp", timestamp);
	Py_DECREF(truepose);
	Py_DECREF(odometrypose);
	Py_DECREF(timestamp);

	$result = sim_truepos;
}


%typemap(out) carmen_arm_state_message*{
        int i;

	PyObject* joint_angles= PyList_New(0);
	for(i=0; i<$1->num_joints; i++){
	  PyObject* pt = PyFloat_FromDouble((double)$1->joint_angles[i]);		
	  PyList_Append(joint_angles,pt);
	  Py_DECREF(pt);
	}

	PyObject* joint_currents= PyList_New(0);
	for(i=0; i<$1->num_currents; i++){
	  PyObject* pt = PyFloat_FromDouble((double)$1->joint_currents[i]);		
	  PyList_Append(joint_currents,pt);
	  Py_DECREF(pt);
	}

	PyObject* joint_angular_vels= PyList_New(0);
	for(i=0; i<$1->num_vels; i++){
	  PyObject* pt = PyFloat_FromDouble((double)$1->joint_angular_vels[i]);		
	  PyList_Append(joint_angular_vels,pt);
	  Py_DECREF(pt);
	}


	PyObject* gripper_closed = PyFloat_FromDouble((double)$1->gripper_closed);	

	PyObject* retObj = PyDict_New();
	PyDict_SetItemString(retObj, "joint_angles", joint_angles);
	PyDict_SetItemString(retObj, "joint_currents", joint_currents);
	PyDict_SetItemString(retObj, "joint_angular_vels", joint_angular_vels);
	PyDict_SetItemString(retObj, "gripper_closed", gripper_closed);

	Py_DECREF(gripper_closed);
	$result = retObj;
}

%typemap(out) carmen_bumblebee_basic_stereoimage_message*{

	PyObject* retObj = PyDict_New();

	PyObject* width = PyInt_FromLong((int)$1->width);
	PyObject* height = PyInt_FromLong((int)$1->height);
	PyObject* image_size = PyInt_FromLong((int)$1->image_size);
	PyObject* isRectified = PyInt_FromLong((int)$1->isRectified);
	PyObject* ts = PyFloat_FromDouble((double)$1->timestamp);

	PyDict_SetItemString(retObj, "timestamp", ts);
	PyDict_SetItemString(retObj, "width", width);
	PyDict_SetItemString(retObj, "height", height);
	PyDict_SetItemString(retObj, "image_size", image_size);
	PyDict_SetItemString(retObj, "isRectified", isRectified);

	Py_DECREF(ts);
	Py_DECREF(width);
	Py_DECREF(height);
	Py_DECREF(image_size);
	Py_DECREF(isRectified);

	PyObject* raw_left = PyList_New(0);
	PyObject* raw_right = PyList_New(0);
    int i;
	for(i=0; i<$1->image_size; i++){
	  PyObject* pt_l = PyInt_FromSize_t((unsigned char)$1->raw_left[i]);
	  PyObject* pt_r = PyInt_FromSize_t((unsigned char)$1->raw_right[i]);
	  PyList_Append(raw_left, pt_l);
	  PyList_Append(raw_right, pt_r);
	  Py_DECREF(pt_l);
	  Py_DECREF(pt_r);
	}

	PyDict_SetItemString(retObj, "raw_left", raw_left);
	PyDict_SetItemString(retObj, "raw_right", raw_right);

	Py_DECREF(raw_left);
	Py_DECREF(raw_right);

	$result = retObj;
}

%include "std_string.i"
%include "typemaps.i"
%include "carrays.i"
%array_class(double, doubleArray);
%array_class(float, floatArray);

/* turn on director wrapping Callback */
%feature("director") MessageHandler;
#define __attribute__(x)
%include "carmen.h"
%include "global.h"
#%include "map.h"

%include "pyCarmen.h"
%include "pyCarmenMessages.h"




%include base_ackerman_messages.h
// %include camera_messages.h
%include gps_nmea_messages.h
%include laser_messages.h
%include localize_ackerman_messages.h
%include logger_messages.h
%include playback_messages.h
%include map_messages.h
%include navigator_ackerman_messages.h
%include param_messages.h
%include robot_ackerman_messages.h
%include simulator_ackerman_messages.h
//%include linemapping.h
//%include linemapping_messages.h


//%include hokuyo_messages.h
//%include pantilt_messages.h
//%include proccontrol_messages.h
//segway_messages.h
%include bumblebee_basic_messages.h
%include gps_xyz_messages.h


%include base_ackerman_interface.h
// %include camera_interface.h
%include gps_nmea_interface.h
%include laser_interface.h
%include localize_ackerman_interface.h
%include playback_interface.h
%include map_interface.h
%include navigator_ackerman_interface.h
%include param_interface.h
%include robot_ackerman_interface.h
%include simulator_ackerman_interface.h
%include ipc.h
%include map_io.h

//%include planner_interface.h
//%include hokuyo_interface.h
//%include pantilt_interface.h
//%include proccontrol_interface.h
//%include segway_interface.h

%typemap(in) __va_list_tag 
%{ 
va_list* tmp_$input = (va_list*)0; 
// instantiate tmp_$input; 

// copy data to tmp variable 
va_copy(tmp_$input,*((va_list *)($input))); 
// assign the argument 
$1 = tmp_$input; 
%}

// This tells SWIG to treat char ** as a special case
%typemap(in) char ** {
  /* Check if is a list */
  if (PyList_Check($input)) {
    int size = PyList_Size($input);
    int i = 0;
    $1 = (char **) malloc((size+1)*sizeof(char *));
    for (i = 0; i < size; i++) {
      PyObject *o = PyList_GetItem($input,i);
      if (PyString_Check(o))
	$1[i] = PyString_AsString(PyList_GetItem($input,i));
      else {
	PyErr_SetString(PyExc_TypeError,"list must contain strings");
	free($1);
	return NULL;
      }
    }
    $1[i] = 0;
  } else {
    PyErr_SetString(PyExc_TypeError,"not a list");
    return NULL;
  }
}

// This cleans up the char ** array we malloc'd before the function call
%typemap(freearg) char ** {
  free((char *) $1);
}

%include "ipc_wrapper.h"
#%include linemapping.h
