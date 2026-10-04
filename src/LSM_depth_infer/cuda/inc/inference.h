#ifndef INFERENCE_H
#define INFERENCE_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>

namespace seek_disparity { class ConfigManager; }

// TrtInferCore - TensorRT 10.x stereo matching inference (batch_size=1)
class TrtInferCore {
public:
    TrtInferCore(const std::string& engine_path, const seek_disparity::ConfigManager* config_manager);
    ~TrtInferCore();
    
    bool initialize();
    void warmup(int iterations = 3);
    
    // Inference: outputs disparity, confidence, resized_rgb and scale factor
    bool infer(const cv::Mat& left, const cv::Mat& right, 
               cv::Mat& disparity, cv::Mat& confidence,
               cv::Mat& resized_rgb, float& scale);

private:
    class Logger : public nvinfer1::ILogger {
    public:
        void log(Severity severity, const char* msg) noexcept override;
    };
    
    struct BindingInfo {
        int index;
        std::string name;
        nvinfer1::Dims dims;
        size_t size;
    };
    
    std::string engine_path_;
    const seek_disparity::ConfigManager* config_manager_;
    Logger logger_;
    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;
    cudaStream_t stream_;
    
    int model_w_, model_h_;
    int camera_img_width_, camera_img_height_;
    int cuda_device_;
    float scale_;  // letterbox scale factor
    
    void* d_left_input_;
    void* d_right_input_;
    uint8_t* d_src_left_;
    uint8_t* d_src_right_;
    uint8_t* d_resized_rgb_;  // resized RGB for point cloud
    uint8_t* h_src_left_;
    uint8_t* h_src_right_;
    uint8_t* h_resized_rgb_;  // host buffer for resized RGB
    std::vector<void*> buffers_;
    
    std::vector<BindingInfo> input_bindings_;
    std::vector<BindingInfo> output_bindings_;
    int valid_width_, valid_height_;
    
    bool loadEngine();
    void allocateBuffers();
    bool setDynamicShapes();
    bool allocateOutputBuffers();
    void calculateValidSize();
    void preprocessGpu(const cv::Mat& left, const cv::Mat& right);
    void postprocess(cv::Mat& disparity, cv::Mat& confidence, cv::Mat& resized_rgb);
};

#endif // INFERENCE_H
