/* 
 * @Author: taifyang
 * @Date: 2024-06-12 09:26:41
 * @LastEditTime: 2026-01-03 20:41:05
 * @Description: source file for YOLO tensorrt inference
 */

#include "yolo_tensorrt.h"

#include <cstdint>

namespace {

bool has_ultralytics_engine_metadata(const std::string& bytes, size_t* metadata_size)
{
	if (bytes.size() <= sizeof(int32_t))
	{
		return false;
	}

	const unsigned char* data = reinterpret_cast<const unsigned char*>(bytes.data());
	int32_t meta_len = static_cast<int32_t>(data[0]) |
		(static_cast<int32_t>(data[1]) << 8) |
		(static_cast<int32_t>(data[2]) << 16) |
		(static_cast<int32_t>(data[3]) << 24);
	if (meta_len <= 0 || meta_len > 1024 * 1024)
	{
		return false;
	}

	size_t prefix_size = sizeof(int32_t) + static_cast<size_t>(meta_len);
	if (prefix_size >= bytes.size())
	{
		return false;
	}

	std::string metadata = bytes.substr(sizeof(int32_t), static_cast<size_t>(meta_len));
	if (metadata.find("\"task\"") == std::string::npos ||
		metadata.find("\"imgsz\"") == std::string::npos ||
		metadata.find("\"names\"") == std::string::npos)
	{
		return false;
	}

	if (metadata_size != nullptr)
	{
		*metadata_size = prefix_size;
	}
	return true;
}

} // namespace

class TRTLogger : public nvinfer1::ILogger
{
public:
	void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept
	{
	}
} logger;

void YOLO_TensorRT::init(const Algo_Type algo_type, const Device_Type device_type, const Model_Type model_type, const std::string model_path)
{
	m_algo_type = algo_type;

	if (device_type != GPU)
	{
		std::cerr << "TensorRT only support GPU!" << std::endl;
		std::exit(-1);
	}

	m_model_type = model_type;
	
	if(!std::filesystem::exists(model_path))
	{
		std::cerr << "model not exists!" << std::endl;
		std::exit(-1);
	}
	std::ifstream file(model_path, std::ios::binary);
	if (!file.good())
	{
		std::cerr << "read model error!" << std::endl;
		std::exit(-1);
	}

    std::stringstream buffer;
    buffer << file.rdbuf();

    std::string stream_model(buffer.str());
	size_t metadata_size = 0;
	if (has_ultralytics_engine_metadata(stream_model, &metadata_size))
	{
		stream_model.erase(0, metadata_size);
	}

	TRTLogger logger;
	m_runtime = nvinfer1::createInferRuntime(logger);
	m_engine = m_runtime->deserializeCudaEngine(stream_model.data(), stream_model.size());
	if (m_engine == nullptr)
	{
		std::cerr << "tensorrt create engine error!" << std::endl;
		std::exit(-1);
	}

	m_execution_context = m_engine->createExecutionContext();
}

void YOLO_TensorRT::release()
{
	cudaFree(m_input_device);

#if NV_TENSORRT_MAJOR < 10
	m_execution_context->destroy();
	m_engine->destroy();
	m_runtime->destroy();
#endif 
}
