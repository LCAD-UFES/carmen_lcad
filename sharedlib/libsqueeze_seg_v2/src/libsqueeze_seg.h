#ifndef LIBSQUEEZESEG_H
#define LIBSQUEEZESEG_H

void
initialize_python_context();

float*
libsqueeze_seg_process_point_cloud(unsigned int number_of_points, float* point_cloud);

void
libsqueeze_seg_test();


#endif
