#include "trt_engine.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <cstring>
#include <cuda_runtime_api.h>

namespace yoloe_infer {

// Logger implementation
void Logger::log(Severity severity, const char* msg) noexcept {
    if (severity != Severity::kINFO) {
        std::cerr << msg << std::endl;
    }
}

// CUDA helpers
void* MultiTextPromptTRTEngine::cuda_malloc(size_t nbytes) {
    void* ptr;
    if (cudaMalloc(&ptr, nbytes) != cudaSuccess) {
        throw std::runtime_error("cudaMalloc failed");
    }
    return ptr;
}

void MultiTextPromptTRTEngine::cuda_free(void* ptr) {
    if (ptr) {
        cudaFree(ptr);
    }
}

void MultiTextPromptTRTEngine::cuda_memcpy_htod(void* dst, const void* src, size_t nbytes) {
    if (cudaMemcpy(dst, src, nbytes, cudaMemcpyHostToDevice) != cudaSuccess) {
        throw std::runtime_error("cudaMemcpy H2D failed");
    }
}

void MultiTextPromptTRTEngine::cuda_memcpy_dtoh(void* dst, const void* src, size_t nbytes) {
    if (cudaMemcpy(dst, src, nbytes, cudaMemcpyDeviceToHost) != cudaSuccess) {
        throw std::runtime_error("cudaMemcpy D2H failed");
    }
}

MultiTextPromptTRTEngine::MultiTextPromptTRTEngine(const std::string& engine_path,
                                                   std::shared_ptr<SimpleTokenizer> tokenizer,
                                                   int num_classes,
                                                   float conf,
                                                   float iou)
    : tokenizer_(tokenizer), num_classes_(num_classes), conf_(conf), iou_(iou) {

    // Loading engine
    // std::cout << "Loading engine: " << engine_path << std::endl;

    // Create runtime and load engine
    runtime_ = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
        throw std::runtime_error("Failed to create TensorRT runtime");
    }

    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
        throw std::runtime_error("Failed to open engine file: " + engine_path);
    }

    engine_file.seekg(0, engine_file.end);
    size_t size = engine_file.tellg();
    engine_file.seekg(0, engine_file.beg);

    std::vector<char> engine_data(size);
    engine_file.read(engine_data.data(), size);
    engine_file.close();

    engine_ = std::unique_ptr<nvinfer1::ICudaEngine>(
        runtime_->deserializeCudaEngine(engine_data.data(), size));
    if (!engine_) {
        throw std::runtime_error("Failed to deserialize engine");
    }

    // Create execution context
    context_ = std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
    if (!context_) {
        throw std::runtime_error("Failed to create execution context");
    }

    // Create CUDA stream
    cudaStreamCreate(&stream_);

    // Setup bindings
    setup_bindings();
}

MultiTextPromptTRTEngine::~MultiTextPromptTRTEngine() {
    // Free CUDA memory
    for (auto& binding : inputs_) {
        cuda_free(binding.device_ptr);
    }
    for (auto& binding : outputs_) {
        cuda_free(binding.device_ptr);
        if (binding.host_ptr) {
            delete[] static_cast<float*>(binding.host_ptr);
        }
    }

    if (stream_) {
        cudaStreamDestroy(stream_);
    }
}

void MultiTextPromptTRTEngine::setup_bindings() {
    inputs_.clear();
    outputs_.clear();

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
        const char* name = engine_->getIOTensorName(i);
        nvinfer1::DataType dtype = engine_->getTensorDataType(name);
        nvinfer1::Dims dims = engine_->getTensorShape(name);
        size_t size = std::accumulate(dims.d, dims.d + dims.nbDims, 1, std::multiplies<size_t>());
        size_t nbytes = size * (dtype == nvinfer1::DataType::kFLOAT ? 4 :
                               dtype == nvinfer1::DataType::kHALF ? 2 :
                               dtype == nvinfer1::DataType::kINT32 ? 4 :
                               dtype == nvinfer1::DataType::kINT64 ? 8 : 1);

        Binding binding;
        binding.name = name;
        binding.dtype = dtype;
        binding.dims = dims;
        binding.size = size;
        binding.nbytes = nbytes;
        binding.is_input = engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;

        if (binding.is_input) {
            binding.device_ptr = cuda_malloc(nbytes);
            inputs_.push_back(binding);

            // Identify input tensors
            std::string name_str(name);
            if (name_str.find("images") != std::string::npos) {
                img_input_idx_ = inputs_.size() - 1;
                img_shape_ = cv::Size(dims.d[3], dims.d[2]); // W, H
            } else {
                // Assume any other input is text tokens, similar to Python
                token_input_idx_ = inputs_.size() - 1;
                // Adjust num_classes based on token shape
                if (dims.d[0] != num_classes_) {
                    num_classes_ = dims.d[0];
                }
            }
        } else {
            binding.device_ptr = cuda_malloc(nbytes);
            binding.host_ptr = new float[size];
            outputs_.push_back(binding);

            int out_idx = outputs_.size() - 1;
            if (dims.nbDims == 4) {
                protos_idx_ = out_idx;
            } else if (dims.nbDims == 3) {
                if (dims.d[1] == 32) {
                    mask_coeffs_idx_ = out_idx;
                } else {
                    preds_idx_ = out_idx;
                }
            }
        }
        
    }
}

void MultiTextPromptTRTEngine::set_prompts(const std::vector<std::string>& prompts) {
    if (prompts == current_prompts_) {
        return;
    }
    current_prompts_ = prompts;
    text_tokens_.resize(num_classes_ * 77, 0);

    // Tokenize each prompt
    for (size_t i = 0; i < prompts.size() && i < (size_t)num_classes_; ++i) {
        std::vector<int> tokens = (*tokenizer_)(prompts[i], 77);

        // Copy tokens to the flat array
        for (size_t j = 0; j < tokens.size() && j < 77; ++j) {
            text_tokens_[i * 77 + j] = static_cast<int64_t>(tokens[j]);
        }
    }
}

std::pair<std::vector<Detection>, Timings> MultiTextPromptTRTEngine::predict(const cv::Mat& img) {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Preprocess
    preprocess(img);
    auto t1 = std::chrono::high_resolution_clock::now();

    // Infer
    infer();
    auto t2 = std::chrono::high_resolution_clock::now();

    // Postprocess
    std::vector<Detection> detections = postprocess();
    auto t3 = std::chrono::high_resolution_clock::now();

    // Calculate timings
    Timings timings;
    timings.pre = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0f;
    timings.infer = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0f;
    timings.post = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1000.0f;

    return {detections, timings};
}

void MultiTextPromptTRTEngine::preprocess(const cv::Mat& img) {
    int h = img_shape_.height;
    int w = img_shape_.width;
    int ih = img.rows;
    int iw = img.cols;

    scale_ = std::min(static_cast<float>(w) / iw, static_cast<float>(h) / ih);
    int nw = static_cast<int>(iw * scale_);
    int nh = static_cast<int>(ih * scale_);

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(nw, nh));

    cv::Mat padded(h, w, CV_8UC3, cv::Scalar(114, 114, 114));
    pad_ = {(w - nw) / 2, (h - nh) / 2};
    resized.copyTo(padded(cv::Rect(pad_.first, pad_.second, nw, nh)));

    orig_shape_ = cv::Size(iw, ih);

    // Convert to float and normalize
    cv::Mat float_img;
    padded.convertTo(float_img, CV_32F, 1.0/255.0);

    // HWC to CHW and transpose
    input_img_buffer_.resize(h * w * 3);
    std::vector<cv::Mat> channels(3);
    cv::split(float_img, channels);
    for (int c = 0; c < 3; ++c) {
        std::memcpy(input_img_buffer_.data() + c * h * w,
                   channels[c].data, h * w * sizeof(float));
    }
}

void MultiTextPromptTRTEngine::infer() {
    // Copy input data to device
    if (img_input_idx_ >= 0) {
        cuda_memcpy_htod(inputs_[img_input_idx_].device_ptr,
                        input_img_buffer_.data(),
                        inputs_[img_input_idx_].nbytes);
    }

    if (token_input_idx_ >= 0) {
        cuda_memcpy_htod(inputs_[token_input_idx_].device_ptr,
                        text_tokens_.data(),
                        inputs_[token_input_idx_].nbytes);
    }

    // Set tensor addresses and enqueue inputs/outputs
    std::vector<void*> bindings;
    for (const auto& input : inputs_) {
        context_->setTensorAddress(input.name.c_str(), input.device_ptr);
        bindings.push_back(input.device_ptr);
    }
    for (const auto& output : outputs_) {
        context_->setTensorAddress(output.name.c_str(), output.device_ptr);
        bindings.push_back(output.device_ptr);
    }

    // Execute using enqueueV3 (recommended for TensorRT 8.5+)
    if (!context_->enqueueV3(stream_)) {
        throw std::runtime_error("Failed to execute inference");
    }

    // Copy output data back
    for (auto& output : outputs_) {
        cuda_memcpy_dtoh(output.host_ptr, output.device_ptr, output.nbytes);
    }

    cudaStreamSynchronize(stream_);
}

std::vector<Detection> MultiTextPromptTRTEngine::postprocess() {
    if (outputs_.empty() || preds_idx_ < 0) {
        return {};
    }

    // Direct access to outputs
    const float* preds = static_cast<const float*>(outputs_[preds_idx_].host_ptr);
    nvinfer1::Dims preds_dims = outputs_[preds_idx_].dims;
    
    const float* protos = nullptr;
    if (protos_idx_ >= 0) {
        protos = static_cast<const float*>(outputs_[protos_idx_].host_ptr);
    }
    
    const float* mask_coeffs_2d = nullptr;
    if (mask_coeffs_idx_ >= 0) {
         mask_coeffs_2d = static_cast<const float*>(outputs_[mask_coeffs_idx_].host_ptr);
    }

    // Reshape predictions to [C, N]
    // TensorRT output is planar: [Batch, Channels, Anchors] -> [1, 46, 8400]
    // The previous code assumed [Batch, Anchors, Channels] (Interleaved)
    int feat_dim = preds_dims.d[1]; 
    int num_anchors = preds_dims.d[2];

    // Parse components
    int num_classes = feat_dim - 4 - 32; // 4 for bbox, 32 for masks
    
    // Pointers to the start of each component in the planar buffer
    // Layout: [bbox (4), scores (num_classes), mask_coeffs (32)] x num_anchors (planar)
    // Actually, it's [4 rows of anchors] followed by [num_classes rows] etc.
    const float* boxes_x = preds + 0 * num_anchors;
    const float* boxes_y = preds + 1 * num_anchors;
    const float* boxes_w = preds + 2 * num_anchors;
    const float* boxes_h = preds + 3 * num_anchors;
    const float* scores_start = preds + 4 * num_anchors;
    
    // Mask coeffs are either from separate output or appended
    const float* mask_coeffs = mask_coeffs_2d ? mask_coeffs_2d :
                              (protos ? preds + (4 + num_classes) * num_anchors : nullptr);

    int num_active = current_prompts_.size();
    if (num_active == 0) {
        return {};
    }

    // Collect all detections
    std::vector<Detection> detections;

    float global_max_score = 0.0f;

    for (int i = 0; i < num_anchors; ++i) {
        // Find best class
        float max_score = 0.0f;
        int best_class = 0;
        
        // Iterate over active classes
        for (int c = 0; c < num_active; ++c) {
            // Planar access: score for class c at anchor i
            float score = scores_start[c * num_anchors + i];
            if (score > max_score) {
                max_score = score;
                best_class = c;
            }
        }
        
        if (max_score > global_max_score) {
            global_max_score = max_score;
        }

        if (max_score < conf_) {
            continue;
        }

        // Extract bbox (planar access)
        float cx = boxes_x[i];
        float cy = boxes_y[i];
        float w = boxes_w[i];
        float h = boxes_h[i];

        // Rescale to original
        cx = (cx - pad_.first) / scale_;
        cy = (cy - pad_.second) / scale_;
        w = w / scale_;
        h = h / scale_;

        // Convert to center format (x, y, w, h)
        Detection det;
        det.bbox = cv::Rect2f(cx - w/2, cy - h/2, w, h);
        det.conf = max_score;
        det.class_id = best_class;
        det.name = current_prompts_[best_class];

        // Extract mask coefficients if available
        if (mask_coeffs && protos) {
            det.mask_coeffs.resize(32);
            for (int k = 0; k < 32; ++k) {
                // mask_coeffs is planar [32, 8400], access by [k * num_anchors + i]
                det.mask_coeffs[k] = mask_coeffs[k * num_anchors + i];
            }
        }

        detections.push_back(det);
    }
    
    // Simple NMS (can be improved)
    if (!detections.empty()) {
        std::sort(detections.begin(), detections.end(),
                 [](const Detection& a, const Detection& b) {
                     return a.conf > b.conf;
                 });

        std::vector<bool> keep(detections.size(), true);
        for (size_t i = 0; i < detections.size(); ++i) {
            if (!keep[i]) continue;
            for (size_t j = i + 1; j < detections.size(); ++j) {
                if (!keep[j]) continue;

                cv::Rect2f inter = detections[i].bbox & detections[j].bbox;
                float inter_area = inter.area();
                float union_area = detections[i].bbox.area() + detections[j].bbox.area() - inter_area;
                float iou = inter_area / (union_area + 1e-6f);

                if (iou > iou_) {
                    keep[j] = false;
                }
            }
        }

        std::vector<Detection> filtered;
        for (size_t i = 0; i < detections.size(); ++i) {
            if (keep[i]) {
                // Generate mask for surviving detection
                if (!detections[i].mask_coeffs.empty() && protos) {
                    int mh = 160;
                    int mw = 160;
                    cv::Mat mask_mat = cv::Mat::zeros(mh, mw, CV_32F);
                    
                    for (int k = 0; k < 32; ++k) {
                        float coeff = detections[i].mask_coeffs[k];
                        const float* proto_layer = protos + k * mh * mw;
                        
                        for (int p = 0; p < mh * mw; ++p) {
                            mask_mat.at<float>(p) += coeff * proto_layer[p];
                        }
                    }
                    
                    // Sigmoid
                    for (int p = 0; p < mh * mw; ++p) {
                         float val = mask_mat.at<float>(p);
                         mask_mat.at<float>(p) = 1.0f / (1.0f + std::exp(-val));
                    }
                    
                    // Resize to network input size (padded)
                    cv::Mat mask_resized;
                    cv::resize(mask_mat, mask_resized, img_shape_);
                    
                    // Crop padding
                    int nw = static_cast<int>(orig_shape_.width * scale_);
                    int nh = static_cast<int>(orig_shape_.height * scale_);
                    
                    cv::Rect valid_rect(pad_.first, pad_.second, nw, nh);
                    valid_rect = valid_rect & cv::Rect(0, 0, mask_resized.cols, mask_resized.rows);
                    
                    if (valid_rect.width > 0 && valid_rect.height > 0) {
                         cv::Mat mask_crop = mask_resized(valid_rect);
                         
                         // Resize to original image size
                         cv::Mat mask_final;
                         cv::resize(mask_crop, mask_final, orig_shape_);
                         
                         // Binarize
                         cv::Mat binary_mask = (mask_final > 0.5);

                         // Crop mask to bounding box (match Python logic)
                         // orig_boxes[i] is already (x, y, w, h) center format in Python, but here detections[i].bbox is (x, y, w, h) top-left format
                         cv::Rect2f bbox = detections[i].bbox;
                         int x1 = std::max(0, static_cast<int>(bbox.x));
                         int y1 = std::max(0, static_cast<int>(bbox.y));
                         int x2 = std::min(orig_shape_.width, static_cast<int>(bbox.x + bbox.width));
                         int y2 = std::min(orig_shape_.height, static_cast<int>(bbox.y + bbox.height));

                         cv::Mat cropped_mask;
                         if (x2 > x1 && y2 > y1) {
                             cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
                             binary_mask(roi).copyTo(cropped_mask);
                         }
                         
                         detections[i].mask = cropped_mask;
                    }
                }
                filtered.push_back(detections[i]);
            }
        }
        detections = filtered;
    }

    return detections;
}

void MultiTextPromptTRTEngine::warmup() {
    cv::Mat dummy(img_shape_.height, img_shape_.width, CV_8UC3, cv::Scalar(0, 0, 0));
    predict(dummy);
}

} // namespace yoloe_infer