#ifndef LIBNEWCRFS_H
#define LIBNEWCRFS_H

void
initialize_python_context_newcrfs();

unsigned char*
libnewcrfs_process_image(int width, int height, unsigned char *image, int cut_param, int down_param);

#endif