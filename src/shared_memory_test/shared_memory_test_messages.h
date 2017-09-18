/*
 * message.h
 *
 *  Created on: Set 19, 2017
 *      Author: Lucas Veronese
 */

#ifndef CARMEN_SHARED_MEMORY_TEST_MESSAGES_H_
#define CARMEN_SHARED_MEMORY_TEST_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	char *shared_memory_name;
	int buffer_index;
	int buffer_size;
	carmen_map_config_t config;
	double timestamp;
	char *host;
} carmen_shared_memory_test_map_message;

#define CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_NAME	"carmen_shared_memory_test_map_message"
#define CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_FMT		"{string, int, int, {int, int, double, [byte:64], string, double, double}, double, string}"

#ifdef __cplusplus
}
#endif

#endif /* CARMEN_SHARED_MEMORY_TEST_MESSAGES_H_ */
