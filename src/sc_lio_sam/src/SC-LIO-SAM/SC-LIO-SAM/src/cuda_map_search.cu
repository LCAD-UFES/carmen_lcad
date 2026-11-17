// cuda_map_search.cu
//
// GPU-accelerated local-map downsample + batched k=5 nearest-neighbor search
// for LIO-SAM's mapOptmization node. See cuda_map_search.h for the calling
// contract. Built for Maxwell/Pascal GTX through Ampere RTX -- see the
// -gencode flags in CMakeLists.txt.
//
// Every entry point checks its CUDA calls and returns false on any error
// instead of crashing, so the caller in mapOptmization.cpp always has a safe
// CPU fallback available for that scan.

#include "cuda_map_search.h"

#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/transform.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>
#include <cstdint>
#include <cstdio>

namespace {

bool checkCuda(cudaError_t err, const char* what)
{
    if (err != cudaSuccess)
    {
        fprintf(stderr, "[cuda_map_search] %s failed: %s\n", what, cudaGetErrorString(err));
        return false;
    }
    return true;
}

bool g_cudaInitialized = false;

// Persistent device-side map buffers. Uploaded once per scan in
// extractCloud() (see mapOptmization_gpu_patch.md), then queried up to 30x
// (once per LM iteration) WITHOUT re-uploading. That reuse -- not just
// "moving the search to the GPU" -- is what actually removes the repeated
// flann searchLevel/addPoint cost seen in perf.
thrust::device_vector<float3> g_cornerMap;
thrust::device_vector<float3> g_surfMap;

// ---------------------------------------------------------------------------
// Voxel-grid centroid reduction, mirroring pcl::VoxelGrid's default centroid
// mode. Built from thrust::sort_by_key + reduce_by_key (both ship with the
// CUDA toolkit) instead of a hand-rolled hash table.
// ---------------------------------------------------------------------------
struct VoxelKeyOp
{
    float invLeaf;
    __host__ __device__
    int64_t operator()(const float4& p) const
    {
        // Bias so negative coordinates (vehicle-centered frames) still pack
        // into a positive key. +/-1,048,576 voxels per axis at a 0.1m leaf
        // is +/-100km of range -- far beyond any local map here.
        const int64_t bias = 1 << 20;
        int64_t ix = (int64_t)floorf(p.x * invLeaf) + bias;
        int64_t iy = (int64_t)floorf(p.y * invLeaf) + bias;
        int64_t iz = (int64_t)floorf(p.z * invLeaf) + bias;
        return (ix << 42) | (iy << 21) | iz;
    }
};

struct Float5 { float x, y, z, i, n; };

struct Float5Add
{
    __host__ __device__
    Float5 operator()(const Float5& a, const Float5& b) const
    {
        return Float5{ a.x + b.x, a.y + b.y, a.z + b.z, a.i + b.i, a.n + b.n };
    }
};

struct ToFloat5
{
    __host__ __device__
    Float5 operator()(const float4& p) const { return Float5{ p.x, p.y, p.z, p.w, 1.0f }; }
};

struct Float5ToCentroid
{
    __host__ __device__
    float4 operator()(const Float5& s) const
    {
        float invN = 1.0f / s.n;
        return make_float4(s.x * invN, s.y * invN, s.z * invN, s.i * invN);
    }
};

// ---------------------------------------------------------------------------
// Brute-force batched k=5 NN. One thread per query point; each thread scans
// the whole map and keeps a running top-5 via linear insertion (k=5 is tiny,
// so this beats a heap). The map -- a few thousand to ~1e4 points after
// downsample -- fits comfortably in L2 cache on GTX and RTX alike, so no
// manual shared-memory tiling is needed to get good throughput.
// ---------------------------------------------------------------------------
__global__ void knn5Kernel(const float3* __restrict__ mapPts, int numMapPts,
                            const float3* __restrict__ queryPts, int numQueryPts,
                            int* __restrict__ idxOut, float* __restrict__ sqDistOut)
{
    int qi = blockIdx.x * blockDim.x + threadIdx.x;
    if (qi >= numQueryPts) return;

    float3 q = queryPts[qi];

    float bestDist[5] = {3.4e38f, 3.4e38f, 3.4e38f, 3.4e38f, 3.4e38f};
    int   bestIdx[5]  = {-1, -1, -1, -1, -1};

    for (int j = 0; j < numMapPts; ++j)
    {
        float3 m = mapPts[j];
        float dx = q.x - m.x, dy = q.y - m.y, dz = q.z - m.z;
        float d = dx * dx + dy * dy + dz * dz;

        if (d < bestDist[4])
        {
            int k = 4;
            while (k > 0 && bestDist[k - 1] > d)
            {
                bestDist[k] = bestDist[k - 1];
                bestIdx[k]  = bestIdx[k - 1];
                --k;
            }
            bestDist[k] = d;
            bestIdx[k]  = j;
        }
    }

    for (int k = 0; k < 5; ++k)
    {
        idxOut[qi * 5 + k]    = bestIdx[k];
        sqDistOut[qi * 5 + k] = bestDist[k];
    }
}

bool knnSearchImpl(const thrust::device_vector<float3>& map,
                    const std::vector<float>& queryXYZ, int numQueryPoints,
                    std::vector<int>& idxOut, std::vector<float>& sqDistOut)
{
    if (!g_cudaInitialized) return false;
    int numMapPts = (int)map.size();
    if (numMapPts < 5 || numQueryPoints <= 0) return false;

    thrust::device_vector<float3> d_query(numQueryPoints);
    // queryXYZ is flat x,y,z floats with the same layout as float3 (no
    // padding on any CUDA-supported ABI), so a direct memcpy is safe.
    cudaError_t err = cudaMemcpy(thrust::raw_pointer_cast(d_query.data()), queryXYZ.data(),
                                  sizeof(float) * 3 * numQueryPoints, cudaMemcpyHostToDevice);
    if (!checkCuda(err, "query upload")) return false;

    thrust::device_vector<int>   d_idx(numQueryPoints * 5);
    thrust::device_vector<float> d_sqDist(numQueryPoints * 5);

    int threads = 128;
    int blocks = (numQueryPoints + threads - 1) / threads;
    knn5Kernel<<<blocks, threads>>>(
        thrust::raw_pointer_cast(map.data()), numMapPts,
        thrust::raw_pointer_cast(d_query.data()), numQueryPoints,
        thrust::raw_pointer_cast(d_idx.data()), thrust::raw_pointer_cast(d_sqDist.data()));

    if (!checkCuda(cudaGetLastError(), "knn5Kernel launch")) return false;
    if (!checkCuda(cudaDeviceSynchronize(), "knn5Kernel sync")) return false;

    idxOut.resize(numQueryPoints * 5);
    sqDistOut.resize(numQueryPoints * 5);
    if (!checkCuda(cudaMemcpy(idxOut.data(), thrust::raw_pointer_cast(d_idx.data()),
                               sizeof(int) * numQueryPoints * 5, cudaMemcpyDeviceToHost),
                    "idx download")) return false;
    if (!checkCuda(cudaMemcpy(sqDistOut.data(), thrust::raw_pointer_cast(d_sqDist.data()),
                               sizeof(float) * numQueryPoints * 5, cudaMemcpyDeviceToHost),
                    "sqDist download")) return false;

    return true;
}

bool uploadMapImpl(thrust::device_vector<float3>& mapBuf,
                    const std::vector<float>& xyz, int numPoints)
{
    if (!g_cudaInitialized) return false;
    if (numPoints < 5) return false;

    mapBuf.resize(numPoints);
    cudaError_t err = cudaMemcpy(thrust::raw_pointer_cast(mapBuf.data()), xyz.data(),
                                  sizeof(float) * 3 * numPoints, cudaMemcpyHostToDevice);
    return checkCuda(err, "map upload");
}

} // anonymous namespace


namespace cuda_map_search {

bool init()
{
    int deviceCount = 0;
    cudaError_t err = cudaGetDeviceCount(&deviceCount);
    if (err != cudaSuccess || deviceCount == 0)
    {
        g_cudaInitialized = false;
        return false;
    }
    err = cudaSetDevice(0);
    g_cudaInitialized = checkCuda(err, "cudaSetDevice(0)");
    return g_cudaInitialized;
}

bool voxelDownsample(const std::vector<float>& xyzi_in, int numPoints,
                      float leafSize, std::vector<float>& xyzi_out)
{
    if (!g_cudaInitialized || numPoints <= 0 || leafSize <= 0.0f) return false;

    thrust::device_vector<float4> d_pts(numPoints);
    if (!checkCuda(cudaMemcpy(thrust::raw_pointer_cast(d_pts.data()), xyzi_in.data(),
                               sizeof(float) * 4 * numPoints, cudaMemcpyHostToDevice),
                    "voxelDownsample upload")) return false;

    thrust::device_vector<int64_t> d_keys(numPoints);
    thrust::transform(thrust::device, d_pts.begin(), d_pts.end(), d_keys.begin(),
                       VoxelKeyOp{ 1.0f / leafSize });

    thrust::sort_by_key(thrust::device, d_keys.begin(), d_keys.end(), d_pts.begin());

    thrust::device_vector<Float5> d_sums(numPoints);
    thrust::transform(thrust::device, d_pts.begin(), d_pts.end(), d_sums.begin(), ToFloat5{});

    thrust::device_vector<int64_t> d_uniqueKeys(numPoints);
    thrust::device_vector<Float5>  d_reducedSums(numPoints);

    auto endPair = thrust::reduce_by_key(thrust::device,
        d_keys.begin(), d_keys.end(), d_sums.begin(),
        d_uniqueKeys.begin(), d_reducedSums.begin(),
        thrust::equal_to<int64_t>(), Float5Add{});

    int numVoxels = (int)(endPair.first - d_uniqueKeys.begin());
    if (numVoxels <= 0) return false;

    thrust::device_vector<float4> d_out(numVoxels);
    thrust::transform(thrust::device, d_reducedSums.begin(), d_reducedSums.begin() + numVoxels,
                       d_out.begin(), Float5ToCentroid{});

    if (!checkCuda(cudaGetLastError(), "voxelDownsample reduce")) return false;

    xyzi_out.resize(numVoxels * 4);
    return checkCuda(cudaMemcpy(xyzi_out.data(), thrust::raw_pointer_cast(d_out.data()),
                                 sizeof(float) * 4 * numVoxels, cudaMemcpyDeviceToHost),
                      "voxelDownsample download");
}

bool uploadCornerMap(const std::vector<float>& xyz, int numPoints)
{ return uploadMapImpl(g_cornerMap, xyz, numPoints); }

bool uploadSurfMap(const std::vector<float>& xyz, int numPoints)
{ return uploadMapImpl(g_surfMap, xyz, numPoints); }

bool knnSearchCorner5(const std::vector<float>& queryXYZ, int numQueryPoints,
                       std::vector<int>& idxOut, std::vector<float>& sqDistOut)
{ return knnSearchImpl(g_cornerMap, queryXYZ, numQueryPoints, idxOut, sqDistOut); }

bool knnSearchSurf5(const std::vector<float>& queryXYZ, int numQueryPoints,
                     std::vector<int>& idxOut, std::vector<float>& sqDistOut)
{ return knnSearchImpl(g_surfMap, queryXYZ, numQueryPoints, idxOut, sqDistOut); }

} // namespace cuda_map_search
