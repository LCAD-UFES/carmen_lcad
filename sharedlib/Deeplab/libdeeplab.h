#ifndef LIBDEEPLAB_H
#define LIBDEEPLAB_H


void
initialize_inference_context();

unsigned char*
process_image(int width, int height, unsigned char *image);

unsigned char*
color_image(int width, int height, unsigned char *seg_map);

unsigned char*
get_label_name_by_number(unsigned char label_number);

int
is_moving_object(unsigned char label_number);


#endif // LIBDEEPLAB_H
