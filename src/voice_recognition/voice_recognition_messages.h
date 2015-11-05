
#ifndef CARMEN_VOICE_RECOGNITION_MESSAGES_H
#define CARMEN_VOICE_RECOGNITION_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct
	{
		int width;
		int height;
		int image_size;
		unsigned char* image;
		double timestamp;
		char *host;
	} carmen_voice_recognition_message;

#define CARMEN_VOICE_RECOGNITION_MESSAGE_NAME	"carmen_voice_recognition_message"
#define CARMEN_VOICE_RECOGNITION_MESSAGE_FMT	"{int, int, int, <ubyte:3>, double, string}"

#ifdef __cplusplus
}
#endif

#endif // CARMEN_VOICE_RECOGNITION_MESSAGES_H

// @}
