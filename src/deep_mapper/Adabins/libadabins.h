#ifndef LIBADABINS_H
#define LIBADABINS_H

void
initialize_python_context();

unsigned char*
libadabins_process_image(int width, int height, unsigned char *image, int cut_param);

#endif
