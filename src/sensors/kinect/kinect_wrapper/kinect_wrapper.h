#ifndef __KINECT_WRAPPER_H
#define __KINECT_WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

	typedef void (*ON_VIDEO_FRAME)(int, uint8_t*, uint32_t, int);
	typedef void (*ON_DEPTH_FRAME)(int, uint16_t*, uint32_t, int);

	void kinect_set_on_video_event(int index, ON_VIDEO_FRAME);
	void kinect_set_on_depth_event(int index, ON_DEPTH_FRAME);

	void kinect_close_manager(void);
	int kinect_device_count(void);

	void kinect_open_device(int index);
	void kinect_close_device(int index);

	void kinect_start_video_capture(int index);
	void kinect_start_depth_capture(int index);

	void kinect_stop_video_capture(int index);
	void kinect_stop_depth_capture(int index);

	void kinect_set_tilt_degrees(int index, double angle);
	double kinect_get_tilt_degrees(int index);

	void kinect_set_led_off(int index);
	void kinect_set_led_green(int index);
	void kinect_set_led_red(int index);
	void kinect_set_led_yellow(int index);
	void kinect_set_led_blink_green(int index);
	void kinect_set_led_blink_red_yellow(int index);

	void kinect_set_video_format_to_RGB(int index);

	void kinect_update_state(int index);

	int kinect_get_depth_width();
	int kinect_get_depth_height();
	int kinect_get_video_width();
	int kinect_get_video_height();

#ifdef __cplusplus
}
#endif

#endif
