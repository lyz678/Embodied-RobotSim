#pragma once

#include <cuda_runtime.h>
#include <iostream>

namespace graspnet_ros {

inline void check_cuda(cudaError_t result, char const *const func, const char *const file, int const line) {
    if (result) {
        std::cerr << "CUDA error at " << file << ":" << line << " code=" << result 
                  << " \"" << func << "\" \n";
    }
}

#define checkCudaErrors(val) graspnet_ros::check_cuda((val), #val, __FILE__, __LINE__)

} // namespace graspnet_ros
