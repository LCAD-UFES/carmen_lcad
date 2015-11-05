
#ifndef CARMEN_STEREO_POINT_CLOUD_MESSAGES_H
#define CARMEN_STEREO_POINT_CLOUD_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	carmen_vector_3D_t *points;
	carmen_vector_3D_t *point_color;
	int num_points;

	double timestamp;
	char *host;

} carmen_stereo_point_cloud_message;
 
#define		CARMEN_STEREO_POINT_CLOUD_NAME       "carmen_stereo_point_cloud_message"
#define		CARMEN_STEREO_POINT_CLOUD_FMT        "{<{double,double,double}:3>,<{double,double,double}:3>,int,double,string}"


#ifdef __cplusplus
}
#endif

#endif
// @}
