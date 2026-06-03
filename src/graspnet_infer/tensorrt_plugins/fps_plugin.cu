#include <cuda_runtime.h>
#include <cuda_fp16.h>
#include <stdio.h>

#define CUDA_CHECK(call) \
    do { \
        cudaError_t error = call; \
        if (error != cudaSuccess) { \
            fprintf(stderr, "CUDA error at %s:%d: %s\n", __FILE__, __LINE__, cudaGetErrorString(error)); \
            exit(1); \
        } \
    } while(0)

// Device function to update distance and index
__device__ void update_dist(int *dists_i, float *dists, int idx1, int idx2) {
    const float v1 = dists[idx1], v2 = dists[idx2];
    const int i1 = dists_i[idx1], i2 = dists_i[idx2];
    dists[idx1] = fmaxf(v1, v2);
    dists_i[idx1] = (v2 > v1) ? i2 : i1;
}

// Furthest Point Sampling kernel
// Input: dataset [B, N, 3]
// Output: idxs [B, npoint]
// Temp: temp [B, N] (for storing minimum distances)
// Helper to fetch float from dataset
__device__ __forceinline__ float fetch_float(const float* data, int idx) {
    return data[idx];
}

__device__ __forceinline__ float fetch_float(const __half* data, int idx) {
    return __half2float(data[idx]);
}

// Furthest Point Sampling kernel
// Input: dataset [B, N, 3]
// Output: idxs [B, npoint]
// Temp: temp [B, N] (for storing minimum distances)
template <typename T, unsigned int block_size>
__global__ void furthest_point_sampling_kernel(
    int b, int n, int npoint,
    const T *__restrict__ dataset,
    float *__restrict__ temp,
    int *__restrict__ idxs) {
    
    if (npoint <= 0) return;
    
    __shared__ float dists[block_size];
    __shared__ int dists_i[block_size];
    
    int batch_index = blockIdx.x;
    dataset += batch_index * n * 3;
    temp += batch_index * n;
    idxs += batch_index * npoint;
    
    int tid = threadIdx.x;
    const int stride = block_size;
    
    // Initialize temp with large values
    for (int k = tid; k < n; k += stride) {
        temp[k] = 1e10f;
    }
    
    
    // Start with point 0
    int old = 0;
    if (tid == 0) idxs[0] = old;
    __syncthreads();
    
    // Iteratively select furthest points
    for (int j = 1; j < npoint; j++) {
        int besti = 0;
        float best = -1.0f;
        float x1 = fetch_float(dataset, old * 3 + 0);
        float y1 = fetch_float(dataset, old * 3 + 1);
        float z1 = fetch_float(dataset, old * 3 + 2);
        
        // Compute distances to all points
        for (int k = tid; k < n; k += stride) {
            float x2 = fetch_float(dataset, k * 3 + 0);
            float y2 = fetch_float(dataset, k * 3 + 1);
            float z2 = fetch_float(dataset, k * 3 + 2);
            
            // Skip zero points
            float mag = x2 * x2 + y2 * y2 + z2 * z2;
            if (mag <= 1e-3f) continue;
            
            // Compute squared distance
            float d = (x2 - x1) * (x2 - x1) + 
                      (y2 - y1) * (y2 - y1) + 
                      (z2 - z1) * (z2 - z1);
            
            // Update minimum distance to any selected point
            float d2 = fminf(d, temp[k]);
            temp[k] = d2;
            
            // Track best (furthest) point
            if (d2 > best) {
                best = d2;
                besti = k;
            }
        }
        
        dists[tid] = best;
        dists_i[tid] = besti;
        __syncthreads();
        
        // Reduction to find global best
        if (block_size >= 512) {
            if (tid < 256) update_dist(dists_i, dists, tid, tid + 256);
            __syncthreads();
        }
        if (block_size >= 256) {
            if (tid < 128) update_dist(dists_i, dists, tid, tid + 128);
            __syncthreads();
        }
        if (block_size >= 128) {
            if (tid < 64) update_dist(dists_i, dists, tid, tid + 64);
            __syncthreads();
        }
        if (block_size >= 64) {
            if (tid < 32) update_dist(dists_i, dists, tid, tid + 32);
            __syncthreads();
        }
        if (block_size >= 32) {
            if (tid < 16) update_dist(dists_i, dists, tid, tid + 16);
            __syncthreads();
        }
        if (block_size >= 16) {
            if (tid < 8) update_dist(dists_i, dists, tid, tid + 8);
            __syncthreads();
        }
        if (block_size >= 8) {
            if (tid < 4) update_dist(dists_i, dists, tid, tid + 4);
            __syncthreads();
        }
        if (block_size >= 4) {
            if (tid < 2) update_dist(dists_i, dists, tid, tid + 2);
            __syncthreads();
        }
        if (block_size >= 2) {
            if (tid < 1) update_dist(dists_i, dists, tid, tid + 1);
            __syncthreads();
        }
        
        old = dists_i[0];
        if (tid == 0) idxs[j] = old;
        __syncthreads();
    }
}

// Wrapper function
// Wrapper function
extern "C" void furthest_point_sampling_cuda(
    int b, int n, int npoint,
    const void *dataset,
    float *temp,
    int *idxs,
    cudaStream_t stream,
    int type) {
    
    unsigned int n_threads = 512;
    if (n < 512) {
        if (n >= 256) n_threads = 256;
        else if (n >= 128) n_threads = 128;
        else if (n >= 64) n_threads = 64;
        else if (n >= 32) n_threads = 32;
        else if (n >= 16) n_threads = 16;
        else if (n >= 8) n_threads = 8;
        else if (n >= 4) n_threads = 4;
        else if (n >= 2) n_threads = 2;
        else n_threads = 1;
    }
    
    if (type == 0) { // FLOAT
        const float* data = static_cast<const float*>(dataset);
        switch (n_threads) {
            case 512: furthest_point_sampling_kernel<float, 512><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 256: furthest_point_sampling_kernel<float, 256><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 128: furthest_point_sampling_kernel<float, 128><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 64: furthest_point_sampling_kernel<float, 64><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 32: furthest_point_sampling_kernel<float, 32><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 16: furthest_point_sampling_kernel<float, 16><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 8: furthest_point_sampling_kernel<float, 8><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 4: furthest_point_sampling_kernel<float, 4><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 2: furthest_point_sampling_kernel<float, 2><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 1: furthest_point_sampling_kernel<float, 1><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            default: furthest_point_sampling_kernel<float, 512><<<b, 512, 0, stream>>>(b, n, npoint, data, temp, idxs);
        }
    } else { // HALF
        const __half* data = static_cast<const __half*>(dataset);
        switch (n_threads) {
            case 512: furthest_point_sampling_kernel<__half, 512><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 256: furthest_point_sampling_kernel<__half, 256><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 128: furthest_point_sampling_kernel<__half, 128><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 64: furthest_point_sampling_kernel<__half, 64><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 32: furthest_point_sampling_kernel<__half, 32><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 16: furthest_point_sampling_kernel<__half, 16><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 8: furthest_point_sampling_kernel<__half, 8><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 4: furthest_point_sampling_kernel<__half, 4><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 2: furthest_point_sampling_kernel<__half, 2><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            case 1: furthest_point_sampling_kernel<__half, 1><<<b, n_threads, 0, stream>>>(b, n, npoint, data, temp, idxs); break;
            default: furthest_point_sampling_kernel<__half, 512><<<b, 512, 0, stream>>>(b, n, npoint, data, temp, idxs);
        }
    }
    
    CUDA_CHECK(cudaGetLastError());
}
