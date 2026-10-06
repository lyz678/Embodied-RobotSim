#ifndef FPS_PLUGIN_H
#define FPS_PLUGIN_H

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvInferPlugin.h>
#include <cuda_runtime.h>
#include <string>
#include <vector>

// CUDA kernel declaration
extern "C" void furthest_point_sampling_cuda(
    int b, int n, int npoint,
    const void *dataset,
    float *temp,
    int *idxs,
    cudaStream_t stream,
    int type); // 0: float, 1: half

namespace nvinfer1 {

class FPSPlugin : public IPluginV2DynamicExt {
public:
    FPSPlugin(int npoint);
    
    FPSPlugin(const void* data, size_t length);
    
    ~FPSPlugin() override = default;
    
    int32_t getNbOutputs() const noexcept override { return 1; }
    
    DimsExprs getOutputDimensions(int32_t outputIndex, const DimsExprs* inputs,
                                  int32_t nbInputs, IExprBuilder& exprBuilder) noexcept override;
    
    int32_t initialize() noexcept override;
    
    void terminate() noexcept override;
    
    size_t getWorkspaceSize(const PluginTensorDesc* inputs, int32_t nbInputs,
                           const PluginTensorDesc* outputs, int32_t nbOutputs) const noexcept override;
    
    int32_t enqueue(const PluginTensorDesc* inputDesc, const PluginTensorDesc* outputDesc,
                    const void* const* inputs, void* const* outputs,
                    void* workspace, cudaStream_t stream) noexcept override;
    
    size_t getSerializationSize() const noexcept override;
    
    void serialize(void* buffer) const noexcept override;
    
    void configurePlugin(const DynamicPluginTensorDesc* in, int32_t nbInputs,
                        const DynamicPluginTensorDesc* out, int32_t nbOutputs) noexcept override;
    
    bool supportsFormatCombination(int32_t pos, const PluginTensorDesc* inOut,
                                   int32_t nbInputs, int32_t nbOutputs) noexcept override;
    
    const char* getPluginType() const noexcept override;
    
    const char* getPluginVersion() const noexcept override;
    
    void destroy() noexcept override;
    
    IPluginV2DynamicExt* clone() const noexcept override;
    
    DataType getOutputDataType(int32_t index, const nvinfer1::DataType* inputTypes,
                               int32_t nbInputs) const noexcept override;
    
    void setPluginNamespace(const char* pluginNamespace) noexcept override;
    
    const char* getPluginNamespace() const noexcept override;
    
private:
    int mNpoint;
    int mN;  // Number of input points
    std::string mPluginNamespace;
};

class FPSPluginCreator : public IPluginCreator {
public:
    FPSPluginCreator();
    
    AsciiChar const* getPluginName() const noexcept override;
    
    AsciiChar const* getPluginVersion() const noexcept override;
    
    PluginFieldCollection const* getFieldNames() noexcept override;
    
    IPluginV2DynamicExt* createPlugin(const char* name, const PluginFieldCollection* fc) noexcept override;
    
    IPluginV2DynamicExt* deserializePlugin(const char* name, const void* serialData,
                                          size_t serialLength) noexcept override;
    
    void setPluginNamespace(AsciiChar const* pluginNamespace) noexcept override;
    
    AsciiChar const* getPluginNamespace() const noexcept override;
    
private:
    static PluginFieldCollection mFC;
    static std::vector<PluginField> mPluginAttributes;
    std::string mNamespace;
};

} // namespace nvinfer1

#endif // FPS_PLUGIN_H
