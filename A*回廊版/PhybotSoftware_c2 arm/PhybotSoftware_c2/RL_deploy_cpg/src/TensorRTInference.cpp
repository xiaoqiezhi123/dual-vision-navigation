#include "../include/TensorRTInference.h"
#include <cstring>

using namespace nvinfer1;

namespace {
nvinfer1::Dims makeConcreteDims(const nvinfer1::Dims& dims) {
    nvinfer1::Dims concrete = dims;
    for (int i = 0; i < concrete.nbDims; ++i) {
        if (concrete.d[i] <= 0) {
            concrete.d[i] = 1;
        }
    }
    return concrete;
}
}  // namespace

TensorRTInference::TensorRTInference() {
    cudaStreamCreate(&m_stream);
}

TensorRTInference::~TensorRTInference() {
    freeBuffers();
    if (m_context) {
        delete m_context;
        m_context = nullptr;
    }
    if (m_engine) {
        delete m_engine;
        m_engine = nullptr;
    }
    if (m_runtime) {
        delete m_runtime;
        m_runtime = nullptr;
    }
    if (m_stream) {
        cudaStreamDestroy(m_stream);
        m_stream = nullptr;
    }
}

int TensorRTInference::getDimsSize(const Dims& dims) const {
    int size = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        size *= (dims.d[i] > 0 ? dims.d[i] : 1);
    }
    return size;
}

void TensorRTInference::printModelInfo() const {
    std::cout << "\n>>>>>>>>>> Model Information <<<<<<<<<<" << std::endl;
    std::cout << "Input name:  " << m_inputName << std::endl;
    std::cout << "Input dims:  [";
    for (int i = 0; i < m_inputDims.nbDims; ++i) {
        std::cout << m_inputDims.d[i];
        if (i < m_inputDims.nbDims - 1) std::cout << ", ";
    }
    std::cout << "] (" << m_inputDim << " elements)" << std::endl;
    std::cout << "Output name: " << m_outputName << std::endl;
    std::cout << "Output dims: [";
    for (int i = 0; i < m_outputDims.nbDims; ++i) {
        std::cout << m_outputDims.d[i];
        if (i < m_outputDims.nbDims - 1) std::cout << ", ";
    }
    std::cout << "] (" << m_outputDim << " elements)" << std::endl;
}

bool TensorRTInference::init(const std::string& onnxPath, bool useFP16) {
    m_useFP16 = useFP16;
    if (!buildEngineFromONNX(onnxPath)) {
        std::cerr << "Failed to build TensorRT engine" << std::endl;
        return false;
    }
    if (!allocateBuffers()) {
        std::cerr << "Failed to allocate CUDA buffers" << std::endl;
        return false;
    }
    printModelInfo();
    return true;
}

bool TensorRTInference::buildEngineFromONNX(const std::string& onnxPath) {
    IBuilder* builder = createInferBuilder(m_logger);
    if (!builder) {
        std::cerr << "Failed to create builder" << std::endl;
        return false;
    }

    IBuilderConfig* config = builder->createBuilderConfig();
    if (!config) {
        std::cerr << "Failed to create config" << std::endl;
        delete builder;
        return false;
    }

    config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 2ULL << 30);
    if (m_useFP16 && builder->platformHasFastFp16()) {
        config->setFlag(BuilderFlag::kFP16);
        std::cout << "FP16 mode enabled" << std::endl;
    }

    constexpr uint32_t kExplicitBatchFlag = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    INetworkDefinition* network = builder->createNetworkV2(kExplicitBatchFlag);
    if (!network) {
        std::cerr << "Failed to create network" << std::endl;
        delete config;
        delete builder;
        return false;
    }

    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, m_logger);
    if (!parser) {
        std::cerr << "Failed to create parser" << std::endl;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    if (!parser->parseFromFile(onnxPath.c_str(), static_cast<int>(ILogger::Severity::kWARNING))) {
        std::cerr << "Failed to parse ONNX: " << onnxPath << std::endl;
        delete parser;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    int numInputs = network->getNbInputs();
    int numOutputs = network->getNbOutputs();
    if (numInputs == 0 || numOutputs == 0) {
        std::cerr << "Network has no inputs or outputs" << std::endl;
        delete parser;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    ITensor* inputTensor = network->getInput(0);
    m_inputName = inputTensor->getName();
    m_inputDims = makeConcreteDims(inputTensor->getDimensions());
    m_inputDim = getDimsSize(m_inputDims);

    ITensor* outputTensor = network->getOutput(0);
    m_outputName = outputTensor->getName();
    m_outputDims = makeConcreteDims(outputTensor->getDimensions());
    m_outputDim = getDimsSize(m_outputDims);

    IOptimizationProfile* profile = builder->createOptimizationProfile();
    if (!profile) {
        std::cerr << "Failed to create optimization profile" << std::endl;
        delete parser;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    if (!profile->setDimensions(m_inputName.c_str(), OptProfileSelector::kMIN, m_inputDims) ||
        !profile->setDimensions(m_inputName.c_str(), OptProfileSelector::kOPT, m_inputDims) ||
        !profile->setDimensions(m_inputName.c_str(), OptProfileSelector::kMAX, m_inputDims)) {
        std::cerr << "Failed to set optimization profile dimensions for input: " << m_inputName << std::endl;
        delete parser;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    if (config->addOptimizationProfile(profile) < 0) {
        std::cerr << "Failed to add optimization profile" << std::endl;
        delete parser;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    IHostMemory* serializedModel = builder->buildSerializedNetwork(*network, *config);
    if (!serializedModel) {
        std::cerr << "Failed to build serialized network" << std::endl;
        delete parser;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    m_runtime = createInferRuntime(m_logger);
    if (!m_runtime) {
        std::cerr << "Failed to create runtime" << std::endl;
        delete serializedModel;
        delete parser;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    m_engine = m_runtime->deserializeCudaEngine(serializedModel->data(), serializedModel->size());
    if (!m_engine) {
        std::cerr << "Failed to deserialize engine" << std::endl;
        delete m_runtime;
        delete serializedModel;
        delete parser;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    m_context = m_engine->createExecutionContext();
    if (!m_context) {
        std::cerr << "Failed to create context" << std::endl;
        delete m_engine;
        delete m_runtime;
        delete serializedModel;
        delete parser;
        delete network;
        delete config;
        delete builder;
        return false;
    }

    m_context->setInputShape(m_inputName.c_str(), m_inputDims);

    delete serializedModel;
    delete parser;
    delete network;
    delete config;
    delete builder;

    return true;
}

bool TensorRTInference::allocateBuffers() {
    size_t inputSize = m_inputDim * sizeof(float);
    size_t outputSize = m_outputDim * sizeof(float);
    cudaError_t err;

    err = cudaMallocHost(reinterpret_cast<void**>(&m_inputHost), inputSize);
    if (err != cudaSuccess) {
        std::cerr << "cudaMallocHost failed for input: " << cudaGetErrorString(err) << std::endl;
        return false;
    }
    err = cudaMallocHost(reinterpret_cast<void**>(&m_outputHost), outputSize);
    if (err != cudaSuccess) {
        std::cerr << "cudaMallocHost failed for output: " << cudaGetErrorString(err) << std::endl;
        cudaFreeHost(m_inputHost);
        return false;
    }
    err = cudaMalloc(&m_inputDevice, inputSize);
    if (err != cudaSuccess) {
        std::cerr << "cudaMalloc failed for input device: " << cudaGetErrorString(err) << std::endl;
        return false;
    }
    err = cudaMalloc(&m_outputDevice, outputSize);
    if (err != cudaSuccess) {
        std::cerr << "cudaMalloc failed for output device: " << cudaGetErrorString(err) << std::endl;
        cudaFree(m_inputDevice);
        return false;
    }

    memset(m_inputHost, 0, inputSize);
    memset(m_outputHost, 0, outputSize);
    return true;
}

void TensorRTInference::freeBuffers() {
    if (m_inputHost) {
        cudaFreeHost(m_inputHost);
        m_inputHost = nullptr;
    }
    if (m_outputHost) {
        cudaFreeHost(m_outputHost);
        m_outputHost = nullptr;
    }
    if (m_inputDevice) {
        cudaFree(m_inputDevice);
        m_inputDevice = nullptr;
    }
    if (m_outputDevice) {
        cudaFree(m_outputDevice);
        m_outputDevice = nullptr;
    }
}

bool TensorRTInference::infer(const std::vector<float>& input, std::vector<float>& output) {
    if (input.size() != static_cast<size_t>(m_inputDim)) {
        std::cerr << "Input size mismatch: expected " << m_inputDim << ", got " << input.size() << std::endl;
        return false;
    }

    cudaError_t err;
    std::memcpy(m_inputHost, input.data(), m_inputDim * sizeof(float));
    err = cudaMemcpyAsync(m_inputDevice, m_inputHost, m_inputDim * sizeof(float), cudaMemcpyHostToDevice, m_stream);
    if (err != cudaSuccess) {
        std::cerr << "cudaMemcpyAsync to device failed: " << cudaGetErrorString(err) << std::endl;
        return false;
    }

    m_context->setTensorAddress(m_inputName.c_str(), m_inputDevice);
    m_context->setTensorAddress(m_outputName.c_str(), m_outputDevice);

    bool success = m_context->enqueueV3(m_stream);
    if (!success) {
        std::cerr << "Failed to enqueue inference" << std::endl;
        return false;
    }

    err = cudaMemcpyAsync(m_outputHost, m_outputDevice, m_outputDim * sizeof(float), cudaMemcpyDeviceToHost, m_stream);
    if (err != cudaSuccess) {
        std::cerr << "cudaMemcpyAsync to host failed: " << cudaGetErrorString(err) << std::endl;
        return false;
    }
    err = cudaStreamSynchronize(m_stream);
    if (err != cudaSuccess) {
        std::cerr << "cudaStreamSynchronize failed: " << cudaGetErrorString(err) << std::endl;
        return false;
    }

    output.resize(m_outputDim);
    std::memcpy(output.data(), m_outputHost, m_outputDim * sizeof(float));

    return true;
}
