#include "kinect_wrapper.h"
#include "kinect_device.hpp"
#include <stdlib.h>

KinectManager kinect_mgr;

int kinect_device_count(void){
	return kinect_mgr.deviceCount();
}

void kinect_open_device(int index){
	kinect_mgr.openDevice(index);
}

void kinect_close_device(int index){
	kinect_mgr.closeDevice(index);
}

void kinect_set_on_video_event(int index, ON_VIDEO_FRAME event){
	kinect_mgr.setOnVideoEvent(index, event);
}

void kinect_set_on_depth_event(int index, ON_DEPTH_FRAME event){
	kinect_mgr.setOnDepthEvent(index, event);
}

void kinect_start_video_capture(int index){
	kinect_mgr.startVideo(index);
}

void kinect_start_depth_capture(int index){
	kinect_mgr.startDepth(index);
}

void kinect_stop_video_capture(int index){
	kinect_mgr.stopVideo(index);
}

void kinect_stop_depth_capture(int index){
	kinect_mgr.stopDepth(index);
}

void kinect_set_tilt_degrees(int index, double angle){
	kinect_mgr.setTiltDegrees(index, angle);
	kinect_mgr.updateState(index);
}

double kinect_get_tilt_degrees(int index){
	return kinect_mgr.getTiltDegrees(index);
}

void kinect_set_led_off(int index){
	kinect_mgr.setLedOff(index);
}

void kinect_set_led_green(int index){
	kinect_mgr.setLedGreen(index);
}

void kinect_set_led_red(int index){
	kinect_mgr.setLedRed(index);
}

void kinect_set_led_yellow(int index){
	kinect_mgr.setLedYellow(index);
}

void kinect_set_led_blink_green(int index){
	kinect_mgr.setLedBlinkGreen(index);
}

void kinect_set_led_blink_red_yellow(int index){
	kinect_mgr.setLedBlinkRedYellow(index);
}

void kinect_set_video_format_to_RGB(int index){
	kinect_mgr.setVideoFormatToRGB(index);
}

void kinect_update_state(int index){
	kinect_mgr.updateState(index);
}

int kinect_get_depth_width(){
	return kinect_mgr.getDepthWidth();
}

int kinect_get_depth_height(){
	return kinect_mgr.getDepthHeight();
}

int kinect_get_video_width(){
	return kinect_mgr.getVideoWidth();
}

int kinect_get_video_height(){
	return kinect_mgr.getVideoHeight();
}
