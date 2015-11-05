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

    typedef struct
    {
        int traffic_light_size; /* image_width * image_height*3 */
        short signals; /* Signals Detects */
        char* state; /* State of Signals */
        double distance;
        unsigned char *traffic_light_image;
        double timestamp;
        char *host;
    } carmen_traffic_light_message;

    typedef struct
    {
        short has_signals;
        carmen_vector_3D_t position;
        double distance;
        double timestamp;
        char *host;
    } carmen_mapping_traffic_light_message;

#define      CARMEN_TRAFFIC_LIGHT_NAME       "carmen_traffic_light_message"
#define      CARMEN_TRAFFIC_LIGHT_FMT        "{int,short,string,double,<ubyte:1>,double,string}"

#define      CARMEN_MAPPING_TRAFFIC_LIGHT_NAME       "carmen_mapping_traffic_light_message"
#define      CARMEN_MAPPING_TRAFFIC_LIGHT_FMT        "{short, {double, double, double}, double, double, string}"


#ifdef __cplusplus
}
#endif

#endif