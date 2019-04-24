
#ifndef CARMEN_WEB_CAM_MESSAGES_H
#define CARMEN_WEB_CAM_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct
	{
		int width;
		int height;
		int image_size;
		char *img_data;
		double timestamp;
		char *host;
	} carmen_web_cam_message;

#define CARMEN_WEB_CAM_MESSAGE_NAME	"carmen_web_cam_message"
#define CARMEN_WEB_CAM_MESSAGE_FMT	"{int,int,int,<byte:3>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
