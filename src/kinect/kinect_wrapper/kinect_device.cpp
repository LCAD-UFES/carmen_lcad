#include "kinect_device.hpp"
#include <pthread.h>

KinectDevice::KinectDevice(freenect_context *_ctx, int _index)
: Freenect::FreenectDevice(_ctx, _index)
{
	id = _index;
#ifdef DEBUG
	freenect_set_log_level(_ctx, FREENECT_LOG_SPEW);
#endif
};

void KinectDevice::VideoCallback(void* _video, uint32_t timestamp) {
	int size = 640 * 480 * 3;
	uint8_t* video = static_cast<uint8_t*>(_video);
	if (on_video) (*on_video)(id, video, timestamp, size);
};

void KinectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
	int size = 640 * 480;
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	if (on_depth) (*on_depth)(id, depth, timestamp, size);
};

void KinectDevice::setOnVideoEvent(ON_VIDEO_FRAME event){
	on_video = event;
};

void KinectDevice::setOnDepthEvent(ON_DEPTH_FRAME event){
	on_depth = event;
};

double KinectDevice::getTiltDegrees(){
	updateState();
	return getState().getTiltDegs();
};

void KinectManager::openDevice(int index){
	device[index] = &createDevice<KinectDevice>(index);
}

void KinectManager::closeDevice(int index){
	deleteDevice(index);
	device[index] = NULL;
}

void KinectManager::updateState(int index){
	if (device[index])
		device[index]->updateState();
}

void KinectManager::setVideoFormat(int index, freenect_video_format Format){
	if (device[index])
		device[index]->setVideoFormat(Format);
}

void KinectManager::setLed(int index, freenect_led_options option){
	if (device[index])
		device[index]->setLed(option);
}

void KinectManager::setOnVideoEvent(int index, ON_VIDEO_FRAME event){
	if (device[index])
		device[index]->setOnVideoEvent(event);
}

void KinectManager::setOnDepthEvent(int index, ON_DEPTH_FRAME event){
	if (device[index])
		device[index]->setOnDepthEvent(event);
}

void KinectManager::startVideo(int index){
	if (device[index])
		device[index]->startVideo();
}

void KinectManager::startDepth(int index){
	if (device[index])
		device[index]->startDepth();
}

void KinectManager::stopVideo(int index){
	if (device[index])
		device[index]->stopVideo();
}

void KinectManager::stopDepth(int index){
	if (device[index])
		device[index]->stopDepth();
}

void KinectManager::setTiltDegrees(int index, double angle){
	if (device[index]){
		device[index]->setTiltDegrees(angle);
		device[index]->updateState();
	}
}

double KinectManager::getTiltDegrees(int index){
	if (device[index])
		return device[index]->getTiltDegrees();
	else
		return -1.0;
}

void KinectManager::setLedOff(int index){
	setLed(index, LED_OFF);
}

void KinectManager::setLedGreen(int index){
	setLed(index, LED_GREEN);
}

void KinectManager::setLedRed(int index){
	setLed(index, LED_RED);
}

void KinectManager::setLedYellow(int index){
	setLed(index, LED_YELLOW);
}

void KinectManager::setLedBlinkGreen(int index){
	setLed(index, LED_BLINK_GREEN);
}

void KinectManager::setLedBlinkRedYellow(int index){
	setLed(index, LED_BLINK_RED_YELLOW);
}

void KinectManager::setVideoFormatToRGB(int index){
	setVideoFormat(index, FREENECT_VIDEO_RGB);
}

int KinectManager::getDepthWidth(){
	return 640;
}
int KinectManager::getDepthHeight(){
	return 480;
}
int KinectManager::getVideoWidth(){
	return 640;
}
int KinectManager::getVideoHeight(){
	return 480;
}

