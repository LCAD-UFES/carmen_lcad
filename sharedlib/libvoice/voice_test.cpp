#include "voice.h"

int main()
{
	char *message = "Hello, I am an autonomous car!";

	while (1)
	{
		carmen_voice_send_alert(message, 1, "en-uk-north");
	}

	return 0;
}
