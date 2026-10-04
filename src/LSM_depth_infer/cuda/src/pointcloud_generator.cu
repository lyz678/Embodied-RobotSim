#include "pointcloud_generator.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>

// CUDA核函数：从深度图生成点云
__global__ void depth_to_pointcloud_kernel(
    const float* depth, const uint8_t* color,
    float fx, float fy, float cx, float cy,
    float* points, int* valid_count,
    int rows, int cols)
{
    int u = blockIdx.x * blockDim.x + threadIdx.x;
    int v = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (u >= cols || v >= rows) return;
    
    int idx = v * cols + u;
    float z = depth[idx];
    
    // 跳过无效深度值 (深度范围检查已在视差转深度时完成)
    if (z <= 0.0f) return;
    
    // 计算相机坐标系中的3D坐标
    float x = (static_cast<float>(u) - cx) * z / fx;
    float y = (static_cast<float>(v) - cy) * z / fy;
    
    // 原子操作获取有效点索引
    int point_idx = atomicAdd(valid_count, 1);
    
    // 存储点云数据 (x, y, z, r, g, b)
    int point_offset = point_idx * 6;
    points[point_offset + 0] = x;
    points[point_offset + 1] = y;
    points[point_offset + 2] = z;
    
    // 颜色信息 (BGR -> RGB)
    int color_idx = idx * 3;
    points[point_offset + 3] = static_cast<float>(color[color_idx + 2]); // R
    points[point_offset + 4] = static_cast<float>(color[color_idx + 1]); // G
    points[point_offset + 5] = static_cast<float>(color[color_idx + 0]); // B
}

// CUDAPointCloudGenerator实现
CUDAPointCloudGenerator::CUDAPointCloudGenerator() 
    : d_depth_(nullptr), d_color_(nullptr), 
      d_points_(nullptr), d_valid_count_(nullptr),
      rows_(0), cols_(0), initialized_(false) {
}

CUDAPointCloudGenerator::~CUDAPointCloudGenerator() {
    cleanup();
}

bool CUDAPointCloudGenerator::initialize(int rows, int cols, float /*scale*/) {
    if (initialized_) {
        return true;
    }
    
    rows_ = rows;
    cols_ = cols;
    if (!allocateGPUMemory()) {
        return false;
    }
    
    initialized_ = true;
    return true;
}

bool CUDAPointCloudGenerator::allocateGPUMemory() {
    size_t depth_size = rows_ * cols_ * sizeof(float);
    size_t color_size = rows_ * cols_ * 3 * sizeof(uint8_t);
    size_t points_size = rows_ * cols_ * 6 * sizeof(float);
    
    cudaError_t error;
    
    error = cudaMalloc(&d_depth_, depth_size);
    if (error != cudaSuccess) {
        printf("Failed to allocate d_depth_: %s\n", cudaGetErrorString(error));
        return false;
    }
    
    error = cudaMalloc(&d_color_, color_size);
    if (error != cudaSuccess) {
        printf("Failed to allocate d_color_: %s\n", cudaGetErrorString(error));
        return false;
    }
    
    error = cudaMalloc(&d_points_, points_size);
    if (error != cudaSuccess) {
        printf("Failed to allocate d_points_: %s\n", cudaGetErrorString(error));
        return false;
    }
    
    error = cudaMalloc(&d_valid_count_, sizeof(int));
    if (error != cudaSuccess) {
        printf("Failed to allocate d_valid_count_: %s\n", cudaGetErrorString(error));
        return false;
    }
    
    return true;
}

void CUDAPointCloudGenerator::freeGPUMemory() {
    if (d_depth_) cudaFree(d_depth_);
    if (d_color_) cudaFree(d_color_);
    if (d_points_) cudaFree(d_points_);
    if (d_valid_count_) cudaFree(d_valid_count_);
    
    d_depth_ = nullptr;
    d_color_ = nullptr;
    d_points_ = nullptr;
    d_valid_count_ = nullptr;
}

bool CUDAPointCloudGenerator::copyResultsFromGPU(PointCloudT::Ptr& cloud) {
    int h_valid_count;
    cudaError_t error = cudaMemcpy(&h_valid_count, d_valid_count_, sizeof(int), cudaMemcpyDeviceToHost);
    if (error != cudaSuccess) {
        printf("Failed to copy valid_count: %s\n", cudaGetErrorString(error));
        return false;
    }
    
    if (h_valid_count > 0) {
        float* h_points = new float[h_valid_count * 6];
        error = cudaMemcpy(h_points, d_points_, h_valid_count * 6 * sizeof(float), cudaMemcpyDeviceToHost);
        if (error != cudaSuccess) {
            printf("Failed to copy points data: %s\n", cudaGetErrorString(error));
            delete[] h_points;
            return false;
        }
        
        // 将GPU结果添加到点云
        for (int i = 0; i < h_valid_count; ++i) {
            PointT pt;
            int offset = i * 6;
            pt.x = h_points[offset + 0];
            pt.y = h_points[offset + 1];
            pt.z = h_points[offset + 2];
            pt.r = static_cast<uint8_t>(h_points[offset + 3]);
            pt.g = static_cast<uint8_t>(h_points[offset + 4]);
            pt.b = static_cast<uint8_t>(h_points[offset + 5]);
            cloud->points.push_back(pt);
        }
        
        delete[] h_points;
    }
    
    return true;
}

void CUDAPointCloudGenerator::cleanup() {
    if (initialized_) {
        freeGPUMemory();
        initialized_ = false;
    }
}

bool CUDAPointCloudGenerator::generatePointCloudFromDepth(
    const cv::Mat& color,
    const cv::Mat& depth,
    const Intrinsic& intrinsic,
    PointCloudT::Ptr& cloud)
{
    if (!initialized_) {
        printf("CUDAPointCloudGenerator not initialized\n");
        return false;
    }
    
    // 验证输入
    if (depth.rows != rows_ || depth.cols != cols_) {
        printf("Depth size mismatch: expected %dx%d, got %dx%d\n",
               cols_, rows_, depth.cols, depth.rows);
        return false;
    }
    
    if (color.rows != rows_ || color.cols != cols_) {
        printf("Color size mismatch: expected %dx%d, got %dx%d\n",
               cols_, rows_, color.cols, color.rows);
        return false;
    }
    
    cudaError_t error;
    
    // 拷贝深度数据到GPU
    error = cudaMemcpy(d_depth_, depth.data, 
                       rows_ * cols_ * sizeof(float), cudaMemcpyHostToDevice);
    if (error != cudaSuccess) {
        printf("Failed to copy depth data: %s\n", cudaGetErrorString(error));
        return false;
    }
    
    // 拷贝颜色数据到GPU
    error = cudaMemcpy(d_color_, color.data, 
                       rows_ * cols_ * 3 * sizeof(uint8_t), cudaMemcpyHostToDevice);
    if (error != cudaSuccess) {
        printf("Failed to copy color data: %s\n", cudaGetErrorString(error));
        return false;
    }
    
    // 清零有效点计数
    int zero = 0;
    error = cudaMemcpy(d_valid_count_, &zero, sizeof(int), cudaMemcpyHostToDevice);
    if (error != cudaSuccess) {
        printf("Failed to reset valid_count: %s\n", cudaGetErrorString(error));
        return false;
    }
    
    // 设置网格和块大小
    dim3 block_size(16, 16);
    dim3 grid_size((cols_ + block_size.x - 1) / block_size.x,
                   (rows_ + block_size.y - 1) / block_size.y);
    
    // 启动CUDA核函数
    depth_to_pointcloud_kernel<<<grid_size, block_size>>>(
        d_depth_, d_color_,
        intrinsic.fx, intrinsic.fy, intrinsic.cx, intrinsic.cy,
        d_points_, d_valid_count_, rows_, cols_);
    
    // 检查CUDA错误
    error = cudaGetLastError();
    if (error != cudaSuccess) {
        printf("CUDA kernel error: %s\n", cudaGetErrorString(error));
        return false;
    }
    
    // 同步GPU操作
    cudaDeviceSynchronize();
    
    // 拷贝结果回CPU
    if (!copyResultsFromGPU(cloud)) {
        return false;
    }
    
    return true;
}
