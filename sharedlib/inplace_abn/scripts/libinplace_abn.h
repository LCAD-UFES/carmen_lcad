#ifndef LIBINPLACEABN_H
#define LIBINPLACEABN_H

void
initialize_python_context();

float*
libinplace_abn_process_image(int width, int height, unsigned char *image);

#endif
