#ifndef CARMEN_VIEWER_3D_MESSAGES_H
#define CARMEN_VIEWER_3D_MESSAGES_H

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct {
  int width;					/* The x dimension of the image in pixels */
  int height;					/* The y dimension of the image in pixels */
  int image_size;				/* width*height*bytes_per_pixel */
  unsigned char *raw_image;
  carmen_pose_3D_t camera_pose;
  carmen_pose_3D_t camera_offset;
  double timestamp;
  char *host;
} carmen_viewer_3D_map_view_message;


#define      CARMEN_VIEWER_3D_MAP_VIEW_NAME       "carmen_viewer_3D_map_view"
#define      CARMEN_VIEWER_3D_MAP_VIEW_FMT        "{int,int,int,<ubyte:3>,{{double,double,double},{double,double,double}},{{double,double,double},{double,double,double}},double,string}"


#ifdef __cplusplus
}
#endif

#endif

// @}
