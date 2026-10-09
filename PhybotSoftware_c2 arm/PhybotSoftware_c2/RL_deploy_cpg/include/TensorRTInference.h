#ifndef TENSORRTINFERENCE_H
#define TENSORRTINFERENCE_H

/* include */
#include "../../ThirdParty/TensorRT/include/NvInfer.h"
#include "../../ThirdParty/TensorRT/include/NvOnnxParser.h"
#include <cuda_runtime_api.h>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>

/* 日志类 */
class Logger : public nvinfer1::ILogger {
public:
    /* 重写父类的 log 方法 */
    void log(Severity severity, const char* msg) noexcept override {
        /* 只打印警告级别及以上的日志 */
        if (severity <= Severity::kWARNING) {
            /* 输出到控制台 */
            std::cout << "[TensorRT] " << msg << std::endl;
        }
    }
};

/* TensorRT 推理器类 */
class TensorRTInference {
public:
    
    TensorRTInference();    /* 构造函数 */
    ~TensorRTInference();   /* 析构函数 */

    /* 初始化 */
    bool init(const std::string& onnxPath, bool useFP16 = false);
    
    /* 执行推理 */
    bool infer(const std::vector<float>& input, std::vector<float>& output);
    
    
    /* 获取输入张量的总元素个数 */
    int getInputDim()               const { return m_inputDim;      }
    
    /* 获取输出张量的总元素个数 */
    int getOutputDim()              const { return m_outputDim;     }   
    
    /* 获取输入张量的名称 */
    std::string getInputName()      const { return m_inputName;     }
    
    /* 获取输出张量的名称 */
    std::string getOutputName()     const { return m_outputName;    }
    
    /* 获取输入张量的完整形状（维度信息） */
    nvinfer1::Dims getInputDims()   const { return m_inputDims;     }
    
    /* 获取输出张量的完整形状（维度信息） */
    nvinfer1::Dims getOutputDims()  const { return m_outputDims;    }
    
    /* 打印模型信息到控制台（输入/输出名称、形状、元素个数等） */
    void printModelInfo() const;

private:
    /* 从 ONNX 文件构建 TensorRT 引擎 */
    bool buildEngineFromONNX(const std::string& onnxPath);
    
    /* 分配 GPU 和 CPU 内存缓冲区 */
    bool allocateBuffers();
    
    /* 释放之前分配的所有 GPU 和 CPU 内存 */
    void freeBuffers();
    
    /* 计算张量的总元素个数 */
    int getDimsSize(const nvinfer1::Dims& dims) const;

private:
    
    /* 日志对象，用于接收 TensorRT 的日志输出 */
    Logger m_logger;
        
    /* 运行时对象，用于反序列化引擎 */
    nvinfer1::IRuntime* m_runtime{nullptr};
    
    /* 引擎对象，包含优化后的模型 */
    nvinfer1::ICudaEngine* m_engine{nullptr};
    
    /* 执行上下文，用于实际运行推理 */
    nvinfer1::IExecutionContext* m_context{nullptr};
    
    /* 输入张量的总元素个数 */
    int m_inputDim{0};
    
    /* 输出张量的总元素个数 */
    int m_outputDim{0};
    
    /* 是否启用 FP16 半精度模式 */
    bool m_useFP16{false};
    
    /* 输入张量的形状 */
    nvinfer1::Dims m_inputDims;
    
    /* 输出张量的形状 */
    nvinfer1::Dims m_outputDims;
    
    /* GPU 设备上的输入缓冲区 */
    void* m_inputDevice{nullptr};
    
    /* GPU 设备上的输出缓冲区 */
    void* m_outputDevice{nullptr};
    
    /* CPU 主机上的输入缓冲区 */
    float* m_inputHost{nullptr};
    
    /* CPU 主机上的输出缓冲区 */
    float* m_outputHost{nullptr};
    
    /* CUDA 流，用于异步执行 CUDA 操作 */
    cudaStream_t m_stream{nullptr};
    
    /* 输入张量的名称 */
    std::string m_inputName;
    
    /* 输出张量的名称 */
    std::string m_outputName;
};

/* 头文件结束 */
#endif // TENSORRTINFERENCE_H
