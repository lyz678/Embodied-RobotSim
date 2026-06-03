#ifndef TRT_ENGINE_HPP
#define TRT_ENGINE_HPP

#include <string>
#include <vector>
#include <memory>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>
#include "simple_tokenizer.hpp"

namespace yoloe_infer {

struct Detection {
    cv::Rect2f bbox; // x, y, w, h
    float conf;
    int class_id;
    std::string name;
    cv::Mat mask; // Optional mask
    std::vector<float> mask_coeffs; // Added for mask generation
};

struct Timings {
    float pre;
    float infer;
    float post;
};

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override;
};

class MultiTextPromptTRTEngine {
public:
    MultiTextPromptTRTEngine(const std::string& engine_path, 
                             std::shared_ptr<SimpleTokenizer> tokenizer,
                             int num_classes = 10, 
                             float conf = 0.25, 
                             float iou = 0.45);
    ~MultiTextPromptTRTEngine();

    void set_prompts(const std::vector<std::string>& prompts);
    std::pair<std::vector<Detection>, Timings> predict(const cv::Mat& img);
    void warmup();

    const std::vector<std::string>& get_current_prompts() const { return current_prompts_; }
    int get_num_classes() const { return num_classes_; }

private:
    void load_engine(const std::string& engine_path);
    void setup_bindings();
    void preprocess(const cv::Mat& img);
    void infer();
    std::vector<Detection> postprocess();
    
    // CUDA helpers
    void* cuda_malloc(size_t nbytes);
    void cuda_free(void* ptr);
    void cuda_memcpy_htod(void* dst, const void* src, size_t nbytes);
    void cuda_memcpy_dtoh(void* dst, const void* src, size_t nbytes);

    // Members
    Logger logger_;
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    cudaStream_t stream_;

    std::shared_ptr<SimpleTokenizer> tokenizer_;
    int num_classes_;
    float conf_;
    float iou_;
    std::vector<std::string> current_prompts_;
    std::vector<int64_t> text_tokens_; // flattened [max_classes, 77]

    // Bindings
    struct Binding {
        std::string name;
        nvinfer1::DataType dtype;
        nvinfer1::Dims dims;
        size_t size;
        size_t nbytes;
        void* device_ptr = nullptr;
        void* host_ptr = nullptr; // For outputs
        bool is_input;
    };
    std::vector<Binding> inputs_;
    std::vector<Binding> outputs_;
    
    int img_input_idx_ = -1;
    int token_input_idx_ = -1;

    int preds_idx_ = -1;
    int protos_idx_ = -1;
    int mask_coeffs_idx_ = -1;

    // Preprocessing params
    float scale_;
    std::pair<int, int> pad_;
    cv::Size orig_shape_;
    cv::Size img_shape_; // Network input shape (h, w)

    // Buffers
    std::vector<float> input_img_buffer_;
};

} // namespace yoloe_infer

#endif // TRT_ENGINE_HPP
