#include <cuda_runtime.h>
#include <cstdio>
#include <stdint.h>
#include <algorithm>
#include "cuda_process.cuh"

// CUDA核函数：执行 BGR->RGB, HWC->CHW, 归一化 和 等比例缩放(Letterbox)
__global__ void preprocess_kernel(const uint8_t* __restrict__ input,
                                  float* __restrict__ output,
                                  int input_width, int input_height,
                                  int model_input_width, int model_input_height,
                                  float scale, int valid_width, int valid_height) {
    // 计算当前线程处理的像素在输出CHW缓冲区中的索引
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int image_idx = blockIdx.z; // Batch中的图像索引

    if (x >= model_input_width || y >= model_input_height) {
        return;
    }

    // ImageNet标准化参数
    const float mean[3] = {0.485f, 0.456f, 0.406f};
    const float std[3] = {0.229f, 0.224f, 0.225f};

    float pixel_r, pixel_g, pixel_b;

    // 检查当前像素是否在有效区域内（只填充右边和底部）
    if (x < valid_width && y < valid_height) {
        // 计算原始图像中的坐标
        float orig_x_f = (x + 0.5f) / scale - 0.5f;
        float orig_y_f = (y + 0.5f) / scale - 0.5f;
        int orig_x = static_cast<int>(orig_x_f);
        int orig_y = static_cast<int>(orig_y_f);

        // 防止越界
        orig_x = max(0, min(orig_x, input_width - 1));
        orig_y = max(0, min(orig_y, input_height - 1));

        // 计算原始HWC BGR图像数据中的索引
        int src_idx = (image_idx * input_height * input_width + orig_y * input_width + orig_x) * 3;

        // BGR -> RGB 并归一化到 [0, 1]
        pixel_r = static_cast<float>(input[src_idx + 2]) / 255.0f;
        pixel_g = static_cast<float>(input[src_idx + 1]) / 255.0f;
        pixel_b = static_cast<float>(input[src_idx + 0]) / 255.0f;

        // 应用ImageNet标准化
        pixel_r = (pixel_r - mean[0]) / std[0];
        pixel_g = (pixel_g - mean[1]) / std[1];
        pixel_b = (pixel_b - mean[2]) / std[2];

    } else {
        // padding区域用灰色填充 (128/255 = 0.502) 并应用标准化
        float gray_value = 128.0f / 255.0f;
        pixel_r = (gray_value - mean[0]) / std[0];
        pixel_g = (gray_value - mean[1]) / std[1];
        pixel_b = (gray_value - mean[2]) / std[2];
    }

    // HWC -> CHW 转换并写入输出缓冲区
    int C = 3;
    int H = model_input_height;
    int W = model_input_width;
    int batch_stride = C * H * W;
    
    output[image_idx * batch_stride + 0 * H * W + y * W + x] = pixel_r;
    output[image_idx * batch_stride + 1 * H * W + y * W + x] = pixel_g;
    output[image_idx * batch_stride + 2 * H * W + y * W + x] = pixel_b;
}

void launch_preprocess_kernel(const uint8_t* d_input,
                            float* d_output,
                            int input_width, int input_height,
                            int model_input_width, int model_input_height,
                            int batch_size,
                            cudaStream_t stream) {
    
    float scale = std::min(static_cast<float>(model_input_width) / input_width,
                           static_cast<float>(model_input_height) / input_height);
    
    int valid_width = static_cast<int>(input_width * scale + 0.5f);
    int valid_height = static_cast<int>(input_height * scale + 0.5f);
    
    if (scale < 0.001f) scale = 0.001f;

    dim3 block(16, 16);
    dim3 grid((model_input_width + block.x - 1) / block.x,
              (model_input_height + block.y - 1) / block.y,
              batch_size);
              
    preprocess_kernel<<<grid, block, 0, stream>>>(d_input, d_output,
                                                  input_width, input_height,
                                                  model_input_width, model_input_height,
                                                  scale, valid_width, valid_height);
}

// CUDA kernel: Resize RGB image (letterbox, no normalization)
__global__ void resize_rgb_kernel(const uint8_t* __restrict__ input,
                                   uint8_t* __restrict__ output,
                                   int input_width, int input_height,
                                   int output_width, int output_height,
                                   float scale, int valid_width, int valid_height) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= output_width || y >= output_height) return;
    
    int out_idx = (y * output_width + x) * 3;
    
    if (x < valid_width && y < valid_height) {
        float orig_x_f = (x + 0.5f) / scale - 0.5f;
        float orig_y_f = (y + 0.5f) / scale - 0.5f;
        int orig_x = max(0, min(static_cast<int>(orig_x_f), input_width - 1));
        int orig_y = max(0, min(static_cast<int>(orig_y_f), input_height - 1));
        
        int src_idx = (orig_y * input_width + orig_x) * 3;
        output[out_idx + 0] = input[src_idx + 0];  // B
        output[out_idx + 1] = input[src_idx + 1];  // G
        output[out_idx + 2] = input[src_idx + 2];  // R
    } else {
        output[out_idx + 0] = 128;  // Gray padding
        output[out_idx + 1] = 128;
        output[out_idx + 2] = 128;
    }
}

void launch_resize_rgb_kernel(const uint8_t* d_input,
                              uint8_t* d_output,
                              int input_width, int input_height,
                              int output_width, int output_height,
                              cudaStream_t stream) {
    float scale = std::min(static_cast<float>(output_width) / input_width,
                           static_cast<float>(output_height) / input_height);
    int valid_width = static_cast<int>(input_width * scale + 0.5f);
    int valid_height = static_cast<int>(input_height * scale + 0.5f);
    
    dim3 block(16, 16);
    dim3 grid((output_width + block.x - 1) / block.x,
              (output_height + block.y - 1) / block.y);
              
    resize_rgb_kernel<<<grid, block, 0, stream>>>(d_input, d_output,
                                                   input_width, input_height,
                                                   output_width, output_height,
                                                   scale, valid_width, valid_height);
}
