#ifndef _CARMEN_VIRTUAL_SCAN_MESSAGES_H_
#define _CARMEN_VIRTUAL_SCAN_MESSAGES_H_

typedef struct
{
	int num_rays;
	double *ranges;
	double *angles;
	double *intensity;
	double timestamp;
	char *host;
} carmen_virtual_scan_message;

#define CARMEN_VIRTUAL_SCAN_MESSAGE_NAME "carmen_virtual_scan_message"

#define CARMEN_VIRTUAL_SCAN_MESSAGE_FMT "{int,<{double}:1>,<{double}:1>,<{double}:1>,double,string}"

#endif

// @}
