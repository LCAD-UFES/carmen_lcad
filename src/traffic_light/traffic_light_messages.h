/*********************************************************
 Traffic Light Module
 *********************************************************/

#ifndef CARMEN_TRAFFIC_LIGHT_MESSAGES_H
#define CARMEN_TRAFFIC_LIGHT_MESSAGES_H

#include "global.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define TRAFFIC_LIGHT_GREEN 0
#define TRAFFIC_LIGHT_RED 1
#define TRAFFIC_LIGHT_YELLOW 2

typedef struct
{
    int x1;
    int x2;
    int y1;
    int y2;
    int color;
} carmen_traffic_light;

typedef struct
{
	int num_traffic_lights;
	carmen_traffic_light *traffic_lights;
	double distance;
	int traffic_light_image_size; /* image_width * image_height * 3 */
	unsigned char *traffic_light_image;
	double timestamp;
	char *host;
} carmen_traffic_light_message;

#define      CARMEN_TRAFFIC_LIGHT_NAME       "carmen_traffic_light_message"
#define      CARMEN_TRAFFIC_LIGHT_FMT        "{int, <{int, int, int, int, int}:1>, double, int, <ubyte:4>, double, string}"


typedef struct
{
	short has_signals;
	carmen_vector_3D_t position;
	double distance;
	double timestamp;
	char *host;
} carmen_mapping_traffic_light_message;

#define      CARMEN_MAPPING_TRAFFIC_LIGHT_NAME       "carmen_mapping_traffic_light_message"
#define      CARMEN_MAPPING_TRAFFIC_LIGHT_FMT        "{short, {double, double, double}, double, double, string}"


#ifdef __cplusplus
}
#endif

#endif
