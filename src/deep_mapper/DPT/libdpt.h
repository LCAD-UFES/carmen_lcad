#ifndef LIBDPT_H
#define LIBDPT_H

void
initialize_python_context_dpt();

unsigned char*
libdpt_process_image(int width, int height, unsigned char *image, int cut_param, int down_param);

#endif
