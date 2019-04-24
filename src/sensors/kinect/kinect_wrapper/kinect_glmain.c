#include "kinect_glview.hpp"
#include "kinect_wrapper.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

double freenect_angle = (0);

void onKeyPressed(unsigned char key, int x, int y)
{
	if (key == 27) {
		kinect_set_led_off(0);
		kinect_stop_depth_capture(0);
		kinect_stop_video_capture(0);
		kinect_close_device(0);
	}
	if (key == '0') {
		kinect_set_led_off(0);
	}
	if (key == '1') {
		kinect_set_led_green(0);
	}
	if (key == '2') {
		kinect_set_led_red(0);
	}
	if (key == '3') {
		kinect_set_led_yellow(0);
	}
	if (key == '4') {
		kinect_set_led_blink_green(0);
	}
	if (key == '5') {
		kinect_set_led_blink_red_yellow(0);
	}
	if (key == 'f') {
		kinect_set_video_format_to_RGB(0);
	}

	if (key == 'w') {
		freenect_angle++;
		if (freenect_angle > 30) {
			freenect_angle = 30;
		}
	}
	if (key == 's' || key == 'd') {
		freenect_angle = 10;
	}
	if (key == 'x') {
		freenect_angle--;
		if (freenect_angle < -30) {
			freenect_angle = -30;
		}
	}
	if (key == 'e') {
		freenect_angle = 10;
	}
	if (key == 'c') {
		freenect_angle = -10;
	}
	kinect_set_tilt_degrees(0,freenect_angle);
}

void onDrawScene() {
	kinect_update_state(0);
	printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f", freenect_angle, kinect_get_tilt_degrees(0));
	fflush(stdout);
}

void shutdown_kinect(int signal __attribute__ ((unused)))
{
	keyPressed(27, 0, 0);
}

int main(int argc, char **argv) {

	signal(SIGINT, shutdown_kinect);
	signal(SIGTERM, shutdown_kinect);

	setDrawSceneHandler(onDrawScene);
	setKeyPressedHandler(onKeyPressed);

	kinect_open_device(0);

	kinect_set_on_video_event(0,VideoCallback);
	kinect_set_on_depth_event(0,DepthCallback);

	kinect_set_tilt_degrees(0,10);

	kinect_start_video_capture(0);
	kinect_start_depth_capture(0);

	kinect_set_led_red(0);

	display(&argc, argv);

	kinect_stop_video_capture(0);
	kinect_stop_depth_capture(0);

	kinect_set_led_off(0);

	kinect_close_device(0);

	return 0;
}
