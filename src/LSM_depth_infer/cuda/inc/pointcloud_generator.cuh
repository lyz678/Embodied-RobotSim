#ifndef CUDA_POINTCLOUD_GENERATOR_H
#define CUDA_POINTCLOUD_GENERATOR_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "defs.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// CUDA点云生成器类
class CUDAPointCloudGenerator {
public:
    CUDAPointCloudGenerator();
    ~CUDAPointCloudGenerator();
    
    // 初始化GPU内存
    bool initialize(int rows, int cols, float scale);
    
    // 从深度图生成点云 (深度范围检查已在视差转深度时完成)
    bool generatePointCloudFromDepth(
        const cv::Mat& color,
        const cv::Mat& depth,
        const Intrinsic& intrinsic,
        PointCloudT::Ptr& cloud);
    
    // 清理GPU内存
    void cleanup();

private:
    // GPU内存指针
    float* d_depth_;
    uint8_t* d_color_;
    float* d_points_;
    int* d_valid_count_;
    
    // 图像尺寸
    int rows_;
    int cols_;
    bool initialized_;
    
    // 私有方法
    bool allocateGPUMemory();
    void freeGPUMemory();
    bool copyResultsFromGPU(PointCloudT::Ptr& cloud);
};

#endif // CUDA_POINTCLOUD_GENERATOR_H
