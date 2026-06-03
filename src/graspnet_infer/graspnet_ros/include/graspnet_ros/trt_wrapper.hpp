#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <NvInfer.h>
#include <cuda_runtime.h>

class TrtWrapper {
public:
    TrtWrapper(const std::string& engine_path, const std::string& plugin_path);
    ~TrtWrapper();

    void infer(void* d_input, int num_point, std::vector<void*>& d_outputs);
    void infer_async(void* d_input, int num_point, std::vector<void*>& d_outputs, cudaStream_t stream);

    int getBindingIndex(const std::string& name);
    std::vector<std::string> getOutputNames() const { return output_names_; }
    std::string getInputName() const { return input_name_; }
    nvinfer1::Dims getTensorShape(const std::string& name) const;

private:
    void* plugin_handle_ = nullptr;
    std::shared_ptr<nvinfer1::IRuntime> runtime_;
    std::shared_ptr<nvinfer1::ICudaEngine> engine_;
    std::shared_ptr<nvinfer1::IExecutionContext> context_;
    
    cudaStream_t stream_;

    std::string input_name_;
    std::vector<std::string> output_names_;
};
