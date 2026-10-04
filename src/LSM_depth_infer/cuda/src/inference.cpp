/**
 * @file inference.cpp
 * @brief TensorRT 10.x stereo matching inference (batch_size=1)
 */

#include "inference.h"
#include "config_manager.h"
#include "cuda_process.cuh"
#include <fstream>
#include <NvInferPlugin.h>

#define CUDA_CHECK(call) \
    do { \
        cudaError_t error = call; \
        if (error != cudaSuccess) { \
            std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ << " - " << cudaGetErrorString(error) << std::endl; \
            abort(); \
        } \
    } while(0)

void TrtInferCore::Logger::log(Severity severity, const char* msg) noexcept {
    if (severity <= Severity::kWARNING) {
        std::cout << "[TensorRT] " << msg << std::endl;
    }
}

TrtInferCore::TrtInferCore(const std::string& engine_path, const seek_disparity::ConfigManager* config_manager) 
    : engine_path_(engine_path), config_manager_(config_manager),
      runtime_(nullptr), engine_(nullptr), context_(nullptr), stream_(nullptr),
      scale_(1.0f),
      d_left_input_(nullptr), d_right_input_(nullptr),
      d_src_left_(nullptr), d_src_right_(nullptr), d_resized_rgb_(nullptr),
      h_src_left_(nullptr), h_src_right_(nullptr), h_resized_rgb_(nullptr),
      valid_width_(0), valid_height_(0) {
    
    const auto& cfg = config_manager_->getTensorRTConfig();
    camera_img_width_ = cfg.camera_img_width;
    camera_img_height_ = cfg.camera_img_height;
    cuda_device_ = cfg.cuda_device;
    
    if (cudaSetDevice(cuda_device_) != cudaSuccess) {
        throw std::runtime_error("Failed to set CUDA device");
    }
    if (cudaStreamCreate(&stream_) != cudaSuccess) {
        throw std::runtime_error("Failed to create CUDA stream");
    }
}

TrtInferCore::~TrtInferCore() {
    int device_count = 0;
    if (cudaGetDeviceCount(&device_count) == cudaSuccess && device_count > 0) {
        cudaSetDevice(cuda_device_);
        if (stream_) cudaStreamSynchronize(stream_);
        
        cudaFree(d_left_input_); cudaFree(d_right_input_);
        cudaFree(d_src_left_); cudaFree(d_src_right_); cudaFree(d_resized_rgb_);
        delete[] h_src_left_; delete[] h_src_right_; delete[] h_resized_rgb_;
        
        for (void* buf : buffers_) if (buf) cudaFree(buf);
        buffers_.clear();
        
        if (stream_) cudaStreamDestroy(stream_);
        cudaDeviceSynchronize();
    }
    delete context_; delete engine_; delete runtime_;
}

bool TrtInferCore::loadEngine() {
    std::ifstream file(engine_path_, std::ios::binary | std::ios::ate);
    if (!file.good()) return false;
    
    size_t size = file.tellg();
    file.seekg(0);
    std::vector<char> data(size);
    file.read(data.data(), size);
    
    runtime_ = nvinfer1::createInferRuntime(logger_);
    if (!runtime_) return false;
    
    engine_ = runtime_->deserializeCudaEngine(data.data(), size);
    return engine_ != nullptr;
}

void TrtInferCore::allocateBuffers() {
    const int n = engine_->getNbIOTensors();
    buffers_.resize(n);
    
    for (int i = 0; i < n; ++i) {
        const char* name = engine_->getIOTensorName(i);
        bool is_input = engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
        nvinfer1::Dims dims = engine_->getTensorShape(name);
        
        // Fix dynamic dims (batch=1)
        if (dims.d[0] == -1) dims.d[0] = 1;
        if (is_input) {
            if (dims.d[1] == -1) dims.d[1] = 3;
            model_h_ = dims.d[2];
            model_w_ = dims.d[3];
        } else {
            if (dims.d[1] == -1) dims.d[1] = 1;
        }
        
        size_t size = sizeof(float);
        for (int j = 0; j < dims.nbDims; ++j) size *= dims.d[j];
        
        CUDA_CHECK(cudaMalloc(&buffers_[i], size));
        
        BindingInfo info = {i, name, dims, size};
        (is_input ? input_bindings_ : output_bindings_).push_back(info);
    }
    
    // Preprocess buffers
    size_t model_size = model_h_ * model_w_ * 3;
    size_t src_size = camera_img_height_ * camera_img_width_ * 3;
    
    CUDA_CHECK(cudaMalloc(&d_left_input_, model_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_right_input_, model_size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_src_left_), src_size));
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_src_right_), src_size));
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_resized_rgb_), model_size));  // resized RGB
    
    h_src_left_ = new uint8_t[src_size];
    h_src_right_ = new uint8_t[src_size];
    h_resized_rgb_ = new uint8_t[model_size];
}

bool TrtInferCore::setDynamicShapes() {
    nvinfer1::Dims dims = {4, {1, 3, model_h_, model_w_}};
    
    for (const auto& b : input_bindings_) {
        if (!context_->setInputShape(b.name.c_str(), dims)) return false;
    }
    
    if (!context_->allInputDimensionsSpecified()) return false;
    return allocateOutputBuffers();
}

bool TrtInferCore::allocateOutputBuffers() {
    for (auto& b : output_bindings_) {
        cudaFree(buffers_[b.index]);
        buffers_[b.index] = nullptr;
        
        nvinfer1::Dims dims = context_->getTensorShape(b.name.c_str());
        size_t size = sizeof(float);
        for (int i = 0; i < dims.nbDims; ++i) {
            if (dims.d[i] <= 0) return false;
            size *= dims.d[i];
        }
        
        CUDA_CHECK(cudaMalloc(&buffers_[b.index], size));
        b.dims = dims;
        b.size = size;
    }
    return true;
}

void TrtInferCore::calculateValidSize() {
    if (output_bindings_.empty()) return;
    
    const auto& d = output_bindings_[0].dims;
    int out_h = (d.nbDims == 4) ? d.d[2] : d.d[1];
    int out_w = (d.nbDims == 4) ? d.d[3] : d.d[2];
    
    scale_ = std::min(float(out_w) / camera_img_width_, float(out_h) / camera_img_height_);
    valid_width_ = std::min(int(camera_img_width_ * scale_ + 0.5f), out_w);
    valid_height_ = std::min(int(camera_img_height_ * scale_ + 0.5f), out_h);
}

bool TrtInferCore::initialize() {
    initLibNvInferPlugins(&logger_, "");
    
    if (!loadEngine()) return false;
    
    context_ = engine_->createExecutionContext();
    if (!context_) return false;
    
    allocateBuffers();
    if (!setDynamicShapes()) return false;
    calculateValidSize();
    
    return runtime_ && engine_ && context_ && stream_ && 
           !input_bindings_.empty() && !output_bindings_.empty();
}

void TrtInferCore::preprocessGpu(const cv::Mat& left, const cv::Mat& right) {
    size_t img_size = left.rows * left.cols * 3;
    
    memcpy(h_src_left_, left.data, img_size);
    memcpy(h_src_right_, right.data, img_size);
    
    CUDA_CHECK(cudaMemcpyAsync(d_src_left_, h_src_left_, img_size, cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_src_right_, h_src_right_, img_size, cudaMemcpyHostToDevice, stream_));
    
    // Preprocess for model input (normalize + CHW)
    launch_preprocess_kernel(d_src_left_, static_cast<float*>(d_left_input_),
                             left.cols, left.rows, model_w_, model_h_, 1, stream_);
    launch_preprocess_kernel(d_src_right_, static_cast<float*>(d_right_input_),
                             right.cols, right.rows, model_w_, model_h_, 1, stream_);
    
    // Resize left RGB for point cloud (no normalization, keep BGR)
    launch_resize_rgb_kernel(d_src_left_, d_resized_rgb_,
                             left.cols, left.rows, valid_width_, valid_height_, stream_);
    
    size_t prep_size = model_w_ * model_h_ * 3 * sizeof(float);
    CUDA_CHECK(cudaMemcpyAsync(buffers_[input_bindings_[0].index], d_left_input_, prep_size, cudaMemcpyDeviceToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(buffers_[input_bindings_[1].index], d_right_input_, prep_size, cudaMemcpyDeviceToDevice, stream_));
}

void TrtInferCore::postprocess(cv::Mat& disparity, cv::Mat& confidence, cv::Mat& resized_rgb) {
    const auto& disp_b = output_bindings_[0];
    std::vector<float> h_disp(disp_b.size / sizeof(float));
    CUDA_CHECK(cudaMemcpyAsync(h_disp.data(), buffers_[disp_b.index], disp_b.size, cudaMemcpyDeviceToHost, stream_));
    
    std::vector<float> h_conf;
    bool has_conf = output_bindings_.size() > 1;
    if (has_conf) {
        const auto& conf_b = output_bindings_[1];
        h_conf.resize(conf_b.size / sizeof(float));
        CUDA_CHECK(cudaMemcpyAsync(h_conf.data(), buffers_[conf_b.index], conf_b.size, cudaMemcpyDeviceToHost, stream_));
    }
    
    // Copy resized RGB from GPU
    size_t rgb_size = valid_height_ * valid_width_ * 3;
    CUDA_CHECK(cudaMemcpyAsync(h_resized_rgb_, d_resized_rgb_, rgb_size, cudaMemcpyDeviceToHost, stream_));
    
    cudaStreamSynchronize(stream_);
    
    int out_h = (disp_b.dims.nbDims == 4) ? disp_b.dims.d[2] : disp_b.dims.d[1];
    int out_w = (disp_b.dims.nbDims == 4) ? disp_b.dims.d[3] : disp_b.dims.d[2];
    int w = std::min(valid_width_, out_w);
    int h = std::min(valid_height_, out_h);
    
    cv::Mat full_disp(out_h, out_w, CV_32F, h_disp.data());
    disparity = full_disp(cv::Rect(0, 0, w, h)).clone();
    
    if (has_conf) {
        cv::Mat full_conf(out_h, out_w, CV_32F, h_conf.data());
        confidence = full_conf(cv::Rect(0, 0, w, h)).clone();
    } else {
        confidence = cv::Mat::ones(h, w, CV_32F);
    }
    
    // Create resized RGB Mat
    resized_rgb = cv::Mat(h, w, CV_8UC3, h_resized_rgb_).clone();
}

bool TrtInferCore::infer(const cv::Mat& left, const cv::Mat& right, 
                         cv::Mat& disparity, cv::Mat& confidence,
                         cv::Mat& resized_rgb, float& scale) {
    preprocessGpu(left, right);
    
    for (const auto& b : input_bindings_)
        context_->setTensorAddress(b.name.c_str(), buffers_[b.index]);
    for (const auto& b : output_bindings_)
        context_->setTensorAddress(b.name.c_str(), buffers_[b.index]);
    
    if (!context_->enqueueV3(stream_)) return false;
    
    postprocess(disparity, confidence, resized_rgb);
    scale = scale_;
    return true;
}

void TrtInferCore::warmup(int iterations) {
    cv::Mat left = cv::Mat::zeros(camera_img_height_, camera_img_width_, CV_8UC3);
    cv::Mat right = cv::Mat::zeros(camera_img_height_, camera_img_width_, CV_8UC3);
    cv::randu(left, cv::Scalar(0,0,0), cv::Scalar(255,255,255));
    cv::randu(right, cv::Scalar(0,0,0), cv::Scalar(255,255,255));
    
    cv::Mat disp, conf, rgb;
    float s;
    for (int i = 0; i < iterations; ++i) {
        infer(left, right, disp, conf, rgb, s);
    }
}
