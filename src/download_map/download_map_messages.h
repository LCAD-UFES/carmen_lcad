
#ifndef CARMEN_DOWNLOAD_MAP_MESSAGES_H
#define CARMEN_DOWNLOAD_MAP_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct
	{
		int height;
		int width;
		int size;
		char *image_data;
		carmen_vector_3D_t position;
		carmen_vector_3D_t map_center;
		double timestamp;
		char *host;
	} carmen_download_map_message;

#define CARMEN_DOWNLOAD_MAP_MESSAGE_NAME "carmen_download_map_message"
#define CARMEN_DOWNLOAD_MAP_MESSAGE_FMT "{int, int, int, <{char}:3>, {double, double, double}, {double, double, double}, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
