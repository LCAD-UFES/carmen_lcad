#include "voice.h"

int main()
{
	char *message = "teste";

	while (1)
	{
		carmen_voice_send_alert(message);
	}

	return 0;
}
