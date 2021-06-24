#ifndef LIBDPT_H
#define LIBDPT_H

void
initialize_python_context();

unsigned char*
libdpt_process_image(int width, int height, unsigned char *image, double timestamp);

#endif
