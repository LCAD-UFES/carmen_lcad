#include <GL/gl.h>
#include <GL/glu.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define TEXTURE_SIZE 512


int create_texture(void);

char *update_map_image_texture(carmen_vector_3D_t car_position, carmen_vector_3D_t gps_position_at_turn_on, double square_size);
char *update_map_image_texture2(carmen_vector_3D_t map_center, double square_size, IplImage *img);

carmen_vector_2D_t get_map_image_tex_coord_min(void);
carmen_vector_2D_t get_map_image_tex_coord_max(void);

#ifdef __cplusplus
}
#endif
