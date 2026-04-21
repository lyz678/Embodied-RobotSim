#include "graspnet_ros/trt_wrapper.hpp"
#include <fstream>
#include <iostream>
#include <dlfcn.h>
#include <NvInferPlugin.h>
#include <cassert>

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity != Severity::kINFO)
            std::cout << "[TRT] " << msg << std::endl;
    }
} gLogger;

TrtWrapper::TrtWrapper(const std::string& engine_path, const std::string& plugin_path) {
    // Load Plugin Library
    plugin_handle_ = dlopen(plugin_path.c_str(), RTLD_LAZY);
    if (!plugin_handle_) {
        std::cerr << "Failed to load plugin from " << plugin_path << ": " << dlerror() << std::endl;
        // Proceeding anyway as it might be statically linked or not needed if plugins are registered otherwise
        // But usually needed.
    } else {
        std::cout << "Loaded plugin: " << plugin_path << std::endl;
    }

    // Register Plugins
    if (!initLibNvInferPlugins(&gLogger, "")) {
         std::cerr << "Failed to init LibNvInferPlugins" << std::endl;
    }

    // Load Engine
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        std::cerr << "Error opening engine file: " << engine_path << std::endl;
        throw std::runtime_error("Engine file load failed");
    }
    
    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);
    std::vector<char> engineData(size);
    file.read(engineData.data(), size);
    file.close();

    runtime_ = std::shared_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(gLogger));
    engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(engineData.data(), size));
    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());

    cudaStreamCreate(&stream_);
    
    // Inspect IO
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
        const char* name = engine_->getIOTensorName(i);
        nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            input_name_ = name;
            std::cout << "Input: " << name << std::endl;
        } else {
            output_names_.push_back(name);
            // nvinfer1::Dims d = engine_->getTensorShape(name);
            // std::cout << "Output: " << name << " Shape: [";
            // for(int j=0; j<d.nbDims; ++j) std::cout << d.d[j] << (j<d.nbDims-1?",":"");
            // std::cout << "]" << std::endl;
        }
    }
}

TrtWrapper::~TrtWrapper() {
    context_.reset();
    engine_.reset();
    runtime_.reset();
    if (plugin_handle_) dlclose(plugin_handle_);
    cudaStreamDestroy(stream_);
}

void TrtWrapper::infer(void* d_input, int num_point, std::vector<void*>& d_outputs) {
    infer_async(d_input, num_point, d_outputs, stream_);
    cudaStreamSynchronize(stream_);
}

void TrtWrapper::infer_async(void* d_input, int num_point, std::vector<void*>& d_outputs, cudaStream_t stream) {
    // Set Input
    context_->setTensorAddress(input_name_.c_str(), d_input);
    // Dynamic Shape: [1, num_point, 3]
    if (!context_->setInputShape(input_name_.c_str(), nvinfer1::Dims3{1, num_point, 3})) {
        std::cerr << "Failed to set input shape to [1, " << num_point << ", 3]" << std::endl;
    } else {
        std::cout << "Set input shape to [1, " << num_point << ", 3]" << std::endl;
    }

    // Set Outputs & Log Shapes
    for (size_t i = 0; i < output_names_.size(); ++i) {
        context_->setTensorAddress(output_names_[i].c_str(), d_outputs[i]);
        
        // nvinfer1::Dims d = context_->getTensorShape(output_names_[i].c_str());
        // std::cout << "Runtime Output: " << output_names_[i] << " Shape: [";
        // for(int j=0; j<d.nbDims; ++j) std::cout << d.d[j] << (j<d.nbDims-1?",":"");
        // std::cout << "]" << std::endl;
    }
    
    // Execute
    context_->enqueueV3(stream);
}

int TrtWrapper::getBindingIndex(const std::string& name) {
    // In TRT 8.5+, binding indices are less relevant with Tensor names, 
    // but for compatibility we can look it up.
    // However, since we use enqueueV3 with tensor names, we don't strictly one.
    return -1; 
}

nvinfer1::Dims TrtWrapper::getTensorShape(const std::string& name) const {
    if (context_) {
        return context_->getTensorShape(name.c_str());
    }
    return engine_->getTensorShape(name.c_str());
}
