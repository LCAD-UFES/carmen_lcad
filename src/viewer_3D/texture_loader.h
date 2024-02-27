#include <GL/gl.h>
#include <GL/glu.h>
#include <stdio.h>

#include <carmen/carmen.h>

#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

#define TEXTURE_SIZE 512
#define REMISSION_MAP_SIZE 350


int create_texture(void);
unsigned int create_texture2(void);

char *update_map_image_texture(carmen_vector_3D_t car_position, carmen_vector_3D_t gps_position_at_turn_on, double square_size);
char *update_map_image_texture2(carmen_vector_3D_t map_center, double square_size, IplImage *img);
char *update_remission_map_image_texture(carmen_vector_3D_t map_center, double square_size, IplImage *img);
char *update_localize_imagepos_texture (carmen_vector_3D_t pose, double square_size, char *image);

carmen_vector_2D_t get_map_image_tex_coord_min(void);
carmen_vector_2D_t get_map_image_tex_coord_max(void);

#ifdef __cplusplus
}
#endif
