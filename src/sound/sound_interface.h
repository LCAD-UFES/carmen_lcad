
#ifndef __MICROPHONE_INTERFACE
#define __MICROPHONE_INTERFACE

	#include <carmen/sound_messages.h>

	#ifdef __cplusplus
	extern "C"
	{
	#endif

		void carmen_microphone_subscribe_message (
				carmen_microphone_message *message,
				carmen_handler_t handler,
				carmen_subscribe_t subscribe_how);

		void carmen_microphone_unsubscribe_message (carmen_handler_t handler);
		void carmen_microphone_define_messages ();

	#ifdef __cplusplus
	}
	#endif

#endif

// @}

