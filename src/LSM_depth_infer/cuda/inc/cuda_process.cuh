#pragma once

void launch_preprocess_kernel(const uint8_t* d_input,
    float* d_output,
    int input_width, int input_height,
    int model_input_width, int model_input_height,
    int batch_size,
    cudaStream_t stream);

// Resize RGB image using letterbox scaling (no normalization)
void launch_resize_rgb_kernel(const uint8_t* d_input,
    uint8_t* d_output,
    int input_width, int input_height,
    int output_width, int output_height,
    cudaStream_t stream);
