#ifndef LIBADABINS_H
#define LIBADABINS_H

void
initialize_python_context_adabins();

unsigned char*
libadabins_process_image(int width, int height, unsigned char *image, int cut_param, int down_param);

#endif
