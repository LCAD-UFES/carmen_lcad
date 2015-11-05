
#ifndef __CARMEN_MICROPHONE_MESSAGES_
#define __CARMEN_MICROPHONE_MESSAGES_

	#ifdef __cplusplus
	extern "C" {
	#endif

		typedef struct
		{
			char *buffer;
			int buffer_size;
			double timestamp;
			char *host;
		} carmen_microphone_message;

		#define CARMEN_MICROPHONE_MESSAGE_NAME	"carmen_microphone_message"
		#define CARMEN_MICROPHONE_MESSAGE_FMT		"{<byte:2>,int,double,string}"

	#ifdef __cplusplus
	}
	#endif

#endif

// @}
