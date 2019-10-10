#ifndef LIBSQUEEZESEG_H
#define LIBSQUEEZESEG_H

void
initialize_python_context();

int*
libsqueeze_seg_process_point_cloud(int vertical_resolution, int shots_to_squeeze, float* point_cloud, double timestamp);

#endif
