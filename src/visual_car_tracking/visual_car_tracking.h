/*********************************************************
	---  visual car tracking Module ---
 **********************************************************/

#ifndef visual_CAR_TRACKING_H
#define visual_CAR_TRACKING_H

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>

#include <unistd.h>



void cascade_car_finder(carmen_bumblebee_basic_stereoimage_message*);


#endif
