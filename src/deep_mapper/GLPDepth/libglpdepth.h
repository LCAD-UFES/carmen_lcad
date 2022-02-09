#ifndef LIBGLPDEPTH_H
#define LIBGLPDEPTH_H

void
initialize_python_context_glpdepth();

unsigned char*
libglpdepth_process_image(int width, int height, unsigned char *image, int cut_param, int down_param);

#endif
