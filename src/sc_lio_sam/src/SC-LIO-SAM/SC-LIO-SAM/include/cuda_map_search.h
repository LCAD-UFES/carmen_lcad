#pragma once
#include <vector>

// GPU-accelerated local-map downsample + batched k=5 nearest-neighbor search,
// used by mapOptmization.cpp to replace the two hottest items measured in
// perf on OT-128 (128-channel) bags in mapping mode:
//   - pcl::VoxelGrid::applyFilter                    (~2.5% of runtime)
//   - flann kdtree search in corner/surfOptimization (~28% of runtime)
//
// Every function here degrades gracefully: on any failure (no GPU, driver
// error, etc.) it returns false, and the caller falls back to the original
// PCL kdtree/VoxelGrid CPU path for that scan. Two implementations exist:
//   - cuda_map_search.cu       -> real CUDA/Thrust implementation
//   - cuda_map_search_stub.cpp -> always-false stub, built when no CUDA
//                                 toolkit is found (see CMakeLists.txt)
// so the package builds and runs correctly even on a machine without a GPU.

namespace cuda_map_search {

// Call once at node startup. Returns true if a CUDA-capable device was
// found and initialized.
bool init();

// ---- Voxel-grid downsample (used for map clouds AND current-scan clouds) --
// xyzi_in / xyzi_out are flattened [x0,y0,z0,i0, x1,y1,z1,i1, ...].
// Produces one centroid point per occupied voxel, matching pcl::VoxelGrid's
// default centroid behavior (xyz + intensity averaged).
bool voxelDownsample(const std::vector<float>& xyzi_in, int numPoints,
                      float leafSize, std::vector<float>& xyzi_out);

// ---- Persistent local-map upload -------------------------------------------
// Call once per scan, right after extractCloud() rebuilds/downsamples the
// local corner/surf map -- NOT once per LM iteration. Replaces whatever map
// was previously on the device.
bool uploadCornerMap(const std::vector<float>& xyz, int numPoints);
bool uploadSurfMap(const std::vector<float>& xyz, int numPoints);

// ---- Batched k=5 NN against the map uploaded above -------------------------
// queryXYZ is flattened [x0,y0,z0, x1,y1,z1, ...], already transformed into
// the map frame (i.e. after pointAssociateToMap). idxOut/sqDistOut come back
// as 5 entries per query, SORTED ASCENDING by distance -- same convention as
// pcl::KdTreeFLANN::nearestKSearch -- so the existing CPU code that reads
// pointSearchInd[k]/pointSearchSqDis[k] doesn't need to change.
// Call this up to 30x per scan (once per LM iteration) against the same
// upload; that reuse is the actual source of the speedup.
bool knnSearchCorner5(const std::vector<float>& queryXYZ, int numQueryPoints,
                       std::vector<int>& idxOut, std::vector<float>& sqDistOut);
bool knnSearchSurf5(const std::vector<float>& queryXYZ, int numQueryPoints,
                     std::vector<int>& idxOut, std::vector<float>& sqDistOut);

} // namespace cuda_map_search
