#ifndef LIBGLPDEPTH_H
#define LIBGLPDEPTH_H

void
initialize_python_context();

unsigned char*
libglpdepth_process_image(int width, int height, unsigned char *image);

#endif
