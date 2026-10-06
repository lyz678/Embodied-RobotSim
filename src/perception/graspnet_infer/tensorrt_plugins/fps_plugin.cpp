#include "fps_plugin.h"
#include <cstring>
#include <iostream>

using namespace nvinfer1;

// Plugin field collection
PluginFieldCollection FPSPluginCreator::mFC{};
std::vector<PluginField> FPSPluginCreator::mPluginAttributes;

// FPSPlugin implementation
FPSPlugin::FPSPlugin(int npoint) : mNpoint(npoint), mN(20000), mPluginNamespace("") {}

FPSPlugin::FPSPlugin(const void* data, size_t length) {
    const char* d = static_cast<const char*>(data);
    mNpoint = *reinterpret_cast<const int*>(d);
    d += sizeof(int);
    mN = *reinterpret_cast<const int*>(d);
    d += sizeof(int);
    mPluginNamespace = d;
}

DimsExprs FPSPlugin::getOutputDimensions(int32_t outputIndex, const DimsExprs* inputs,
                                         int32_t nbInputs, IExprBuilder& exprBuilder) noexcept {
    // Input: [B, N, 3] or [B, 3, N]
    // Output: [B, npoint] (indices)
    DimsExprs output;
    output.nbDims = 2;
    output.d[0] = inputs[0].d[0];  // batch size
    output.d[1] = exprBuilder.constant(mNpoint);
    return output;
}

int32_t FPSPlugin::initialize() noexcept {
    return 0;
}

void FPSPlugin::terminate() noexcept {}

size_t FPSPlugin::getWorkspaceSize(const PluginTensorDesc* inputs, int32_t nbInputs,
                                   const PluginTensorDesc* outputs, int32_t nbOutputs) const noexcept {
    // Need temp buffer: [B, N] for storing minimum distances
    int batchSize = inputs[0].dims.d[0];
    return static_cast<size_t>(batchSize) * mN * sizeof(float);
}

int32_t FPSPlugin::enqueue(const PluginTensorDesc* inputDesc, const PluginTensorDesc* outputDesc,
                           const void* const* inputs, void* const* outputs,
                           void* workspace, cudaStream_t stream) noexcept {
    const void* dataset = inputs[0];
    int* idxs = static_cast<int*>(outputs[0]);
    float* temp = static_cast<float*>(workspace);
    
    // Get batch size from input descriptor
    int batchSize = inputDesc[0].dims.d[0];
    
    // Determine N from input dimensions dynamically to be safe
    int n = 0;
    if (inputDesc[0].dims.d[1] == 3) {
        n = inputDesc[0].dims.d[2];
    } else {
        n = inputDesc[0].dims.d[1];
    }

    // Determine type
    int type = (inputDesc[0].type == DataType::kHALF) ? 1 : 0;
    
    // Call CUDA kernel
    furthest_point_sampling_cuda(batchSize, n, mNpoint, dataset, temp, idxs, stream, type);
    
    return 0;
}

size_t FPSPlugin::getSerializationSize() const noexcept {
    return sizeof(int) * 2 + mPluginNamespace.size() + 1;
}

void FPSPlugin::serialize(void* buffer) const noexcept {
    char* d = static_cast<char*>(buffer);
    *reinterpret_cast<int*>(d) = mNpoint;
    d += sizeof(int);
    *reinterpret_cast<int*>(d) = mN;
    d += sizeof(int);
    std::strcpy(d, mPluginNamespace.c_str());
}

void FPSPlugin::configurePlugin(const DynamicPluginTensorDesc* in, int32_t nbInputs,
                                const DynamicPluginTensorDesc* out, int32_t nbOutputs) noexcept {
    // Configure based on input tensor descriptions
    // Input is [B, 3, N] or [B, N, 3] format
    // Determine N from input dimensions
    if (nbInputs > 0 && in[0].desc.dims.nbDims >= 2) {
        // Assuming format [B, C, N] or [B, N, C]
        if (in[0].desc.dims.d[1] == 3) {
            // Format: [B, 3, N]
            mN = in[0].desc.dims.d[2];
        } else {
            // Format: [B, N, 3]
            mN = in[0].desc.dims.d[1];
        }
    }
}

bool FPSPlugin::supportsFormatCombination(int32_t pos, const PluginTensorDesc* inOut,
                                         int32_t nbInputs, int32_t nbOutputs) noexcept {
    if (pos == 0) {
        // Input: float32 or float16
        return (inOut[pos].type == DataType::kFLOAT || inOut[pos].type == DataType::kHALF) && 
               inOut[pos].format == TensorFormat::kLINEAR;
    } else if (pos == 1) {
        // Output: int32
        return inOut[pos].type == DataType::kINT32 && 
               inOut[pos].format == TensorFormat::kLINEAR;
    }
    return false;
}

const char* FPSPlugin::getPluginType() const noexcept {
    return "FPSPlugin";
}

const char* FPSPlugin::getPluginVersion() const noexcept {
    return "1";
}

void FPSPlugin::destroy() noexcept {
    delete this;
}

IPluginV2DynamicExt* FPSPlugin::clone() const noexcept {
    auto* plugin = new FPSPlugin(mNpoint);
    plugin->mN = mN;
    plugin->setPluginNamespace(mPluginNamespace.c_str());
    return plugin;
}

DataType FPSPlugin::getOutputDataType(int32_t index, const DataType* inputTypes,
                                      int32_t nbInputs) const noexcept {
    return DataType::kINT32;
}

void FPSPlugin::setPluginNamespace(const char* pluginNamespace) noexcept {
    mPluginNamespace = pluginNamespace;
}

const char* FPSPlugin::getPluginNamespace() const noexcept {
    return mPluginNamespace.c_str();
}

// FPSPluginCreator implementation
FPSPluginCreator::FPSPluginCreator() {
    mPluginAttributes.clear();
    mPluginAttributes.emplace_back(PluginField("npoint", nullptr, PluginFieldType::kINT32, 1));
    mFC.nbFields = mPluginAttributes.size();
    mFC.fields = mPluginAttributes.data();
}

AsciiChar const* FPSPluginCreator::getPluginName() const noexcept {
    return "FPSPlugin";
}

AsciiChar const* FPSPluginCreator::getPluginVersion() const noexcept {
    return "1";
}

PluginFieldCollection const* FPSPluginCreator::getFieldNames() noexcept {
    return &mFC;
}

IPluginV2DynamicExt* FPSPluginCreator::createPlugin(const char* name,
                                                     const PluginFieldCollection* fc) noexcept {
    const PluginField* fields = fc->fields;
    int npoint = 0;
    
    for (int i = 0; i < fc->nbFields; ++i) {
        if (strcmp(fields[i].name, "npoint") == 0) {
            npoint = *static_cast<const int*>(fields[i].data);
        }
    }
    
    FPSPlugin* plugin = new FPSPlugin(npoint);
    plugin->setPluginNamespace(mNamespace.c_str());
    return plugin;
}

IPluginV2DynamicExt* FPSPluginCreator::deserializePlugin(const char* name,
                                                          const void* serialData,
                                                          size_t serialLength) noexcept {
    FPSPlugin* plugin = new FPSPlugin(serialData, serialLength);
    plugin->setPluginNamespace(mNamespace.c_str());
    return plugin;
}

void FPSPluginCreator::setPluginNamespace(AsciiChar const* pluginNamespace) noexcept {
    mNamespace = pluginNamespace;
}

AsciiChar const* FPSPluginCreator::getPluginNamespace() const noexcept {
    return mNamespace.c_str();
}

// Register plugin
REGISTER_TENSORRT_PLUGIN(FPSPluginCreator);
