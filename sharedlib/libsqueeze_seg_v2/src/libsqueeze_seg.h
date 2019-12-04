#ifndef LIBSQUEEZESEG_H
#define LIBSQUEEZESEG_H

void
initialize_python_context();

long long int*
libsqueeze_seg_process_point_cloud(int vertical_resolution, int shots_to_squeeze, double* point_cloud, double timestamp);

#endif
