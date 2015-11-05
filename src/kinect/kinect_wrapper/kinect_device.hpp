#ifndef __KINECT_DEVICE_HPP
#define __KINECT_DEVICE_HPP

#include <libfreenect.hpp>

typedef void (*ON_VIDEO_FRAME)(int, uint8_t*, uint32_t, int);
typedef void (*ON_DEPTH_FRAME)(int, uint16_t*, uint32_t, int);

class KinectDevice : public Freenect::FreenectDevice {
public:
	KinectDevice(freenect_context *_ctx, int _index);

	void VideoCallback(void* _video, uint32_t timestamp) ;

	void DepthCallback(void* _depth, uint32_t timestamp) ;

	void setOnVideoEvent(ON_VIDEO_FRAME event);

	void setOnDepthEvent(ON_DEPTH_FRAME event);

	double getTiltDegrees();

private:
	ON_VIDEO_FRAME on_video;
	ON_DEPTH_FRAME on_depth;
	int id;
};

class KinectManager : public Freenect::Freenect {
private:
	void setVideoFormat(int index, freenect_video_format format);
	void setLed(int index, freenect_led_options option);

public:
	void updateState(int index);
	int getNumDevices(int index);

	void setOnVideoEvent(int index, ON_VIDEO_FRAME event);
	void setOnDepthEvent(int index, ON_DEPTH_FRAME event);

	void openDevice(int index);
	void closeDevice(int index);

	void startVideo(int index);
	void startDepth(int index);

	void stopVideo(int index);
	void stopDepth(int index);

	void setTiltDegrees(int index, double angle);
	double getTiltDegrees(int index);

	void setLedOff(int index);
	void setLedGreen(int index);
	void setLedRed(int index);
	void setLedYellow(int index);
	void setLedBlinkGreen(int index);
	void setLedBlinkRedYellow(int index);
	void setVideoFormatToRGB(int index);

	int getDepthWidth();
	int getDepthHeight();
	int getVideoWidth();
	int getVideoHeight();

private:
	KinectDevice* device[2];
};

#endif
