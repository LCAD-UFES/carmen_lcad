#include <carmen/carmen.h>
#ifndef CARMEN_MESSAGE_EXAMPLE_H
#define CARMEN_MESSAGE_EXAMPLE_H

#ifdef __cplusplus
extern "C"
{
#endif


#define MAXSIZE 1024

typedef struct
{
	char *message;
} carmen_string_example;

#define CARMEN_STRING_EXAMPLE_NAME	"carmen_string_example"
#define CARMEN_STRING_EXAMPLE_FMT		"{string}"

typedef struct
{
	int num_message;
	carmen_string_example content_message;
	double timestamp;
	char *host;                 /**< The host from which this message was sent **/
} carmen_string_example_message;

#define      CARMEN_STRING_EXAMPLE_MESSAGE_NAME         "carmen_string_example_message"
#define      CARMEN_STRING_EXAMPLE_MESSAGE_FMT          "{int,<{double,double,double,double,double,double}:1>,double,string}"

#ifdef __cplusplus
}
#endif

#endif
