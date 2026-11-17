// cuda_map_search_stub.cpp
//
// Built instead of cuda_map_search.cu when no CUDA toolkit is found on the
// build machine (see CMakeLists.txt). Every function returns false, which
// makes mapOptmization.cpp's fallback path -- the original PCL kdtree /
// VoxelGrid code -- run unconditionally. This lets the package build and run
// correctly on machines without a GPU (e.g. a dev laptop), and on any
// vehicle where the CUDA driver fails to initialize at runtime.

#include "cuda_map_search.h"

namespace cuda_map_search {

bool init() { return false; }

bool voxelDownsample(const std::vector<float>&, int, float, std::vector<float>&)
{ return false; }

bool uploadCornerMap(const std::vector<float>&, int) { return false; }
bool uploadSurfMap(const std::vector<float>&, int) { return false; }

bool knnSearchCorner5(const std::vector<float>&, int, std::vector<int>&, std::vector<float>&)
{ return false; }

bool knnSearchSurf5(const std::vector<float>&, int, std::vector<int>&, std::vector<float>&)
{ return false; }

} // namespace cuda_map_search
