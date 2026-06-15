#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <chrono>
#include <cstdint>
#include <cstring>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <CLI/CLI.hpp>
#include <zenoh.h>

#include "inference_backend_base.hpp"
#include "onnx_runtime_backend.hpp"
#include "tensorrt_backend.hpp"
#include "masks_visualization_engine.hpp"
#include "masks_visualization_kernels.hpp"
#include "depth_visualization_engine.hpp"
#include "fps_timer.hpp"

using namespace cv; 
using namespace std; 

using namespace autoware_pov::vision;

#define VIDEO_INPUT_KEYEXPR "video/input"
#define VIDEO_OUTPUT_KEYEXPR "video/output"
#define DEFAULT_BACKEND "onnxruntime"
#define DEFAULT_PRECISION "cuda"
#define DEFAULT_GPU_ID 0
#define MODEL_TYPE "segmentation"

#define BENCHMARK_OUTPUT_FREQUENCY 100
#define RECV_BUFFER_SIZE 100

struct Time {
    int32_t sec;
    uint32_t nanosec;
};

// std_msgs/msg/Header
struct Header {
    Time stamp;
    std::string frame_id;
};

// sensor_msgs/msg/Image
struct RosImage {
    Header header;
    uint32_t height;
    uint32_t width;
    std::string encoding;
    uint8_t is_bigendian;
    uint32_t step;
    std::vector<uint8_t> data;
};

class CdrDeserializer {
private:
    const uint8_t* buffer;
    size_t size;
    size_t offset;

    template<typename T>
    T read() {
        size_t alignment = sizeof(T);
        size_t padding = (alignment - (offset % alignment)) % alignment;
        offset += padding;

        if (offset + sizeof(T) > size) {
            throw std::runtime_error("CDR Deserializer: read past end of buffer.");
        }

        T value;
        std::memcpy(&value, buffer + offset, sizeof(T));
        offset += sizeof(T);
        return value;
    }

    std::string read_string() {
        uint32_t length = read<uint32_t>();
        if (offset + length > size) {
            throw std::runtime_error("CDR Deserializer: string read past end of buffer.");
        }
        std::string s(reinterpret_cast<const char*>(buffer + offset),
                      length > 0 ? length - 1 : 0);
        offset += length;
        return s;
    }

    std::vector<uint8_t> read_octet_sequence() {
        uint32_t length = read<uint32_t>();
        if (offset + length > size) {
            throw std::runtime_error("CDR Deserializer: vector read past end of buffer.");
        }
        std::vector<uint8_t> vec(buffer + offset, buffer + offset + length);
        offset += length;
        return vec;
    }

public:
    CdrDeserializer(const uint8_t* buf, size_t s) : buffer(buf), size(s), offset(0) {
        if (size < 4) {
            throw std::runtime_error("CDR Deserializer: buffer too small.");
        }
        // skip CDR header
        offset = 4;
    }

    RosImage deserialize_image() {
        RosImage img;
        img.header.stamp.sec     = read<int32_t>();
        img.header.stamp.nanosec = read<uint32_t>();
        img.header.frame_id      = read_string();
        img.height       = read<uint32_t>();
        img.width        = read<uint32_t>();
        img.encoding     = read_string();
        img.is_bigendian = read<uint8_t>();
        img.step         = read<uint32_t>();
        img.data         = read_octet_sequence();
        return img;
    }
};

cv::Mat decode_frame_from_sample(const z_owned_sample_t& sample) {
    const z_loaned_sample_t* loaned_sample = z_loan(sample);
    z_owned_slice_t zslice;
    if (Z_OK != z_bytes_to_slice(z_sample_payload(loaned_sample), &zslice)) {
        throw std::runtime_error("Wrong payload");
    }

    const uint8_t* buf = z_slice_data(z_loan(zslice));
    size_t len = z_slice_len(z_loan(zslice));

    if (!buf || len == 0) {
        z_drop(z_move(zslice));
        throw std::runtime_error("Empty payload");
    }

    CdrDeserializer des(buf, len);
    RosImage image = des.deserialize_image();

    int cv_type = -1;
    if (image.encoding == "bgra8") {
        cv_type = CV_8UC4;
    } else if (image.encoding == "bgr8") {
        cv_type = CV_8UC3;
    } else {
        z_drop(z_move(zslice));
        throw std::runtime_error("Unsupported encoding: " + image.encoding);
    }

    if (image.data.empty()) {
        z_drop(z_move(zslice));
        throw std::runtime_error("Image data is empty");
    }

    cv::Mat frame(image.height, image.width, cv_type);
    size_t expected_size = static_cast<size_t>(image.height) * image.step;
    if (image.data.size() < expected_size) {
        z_drop(z_move(zslice));
        throw std::runtime_error("Image data size mismatch");
    }

    std::memcpy(frame.data, image.data.data(), expected_size);

    z_drop(z_move(zslice));

    return frame;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    CLI::App app{"Carla video scene segmentation visualizer"};
    std::string model_path;
    // Add options
    app.add_option("model_path", model_path, "Path to the ONNX model file")->required()->check(CLI::ExistingFile);
    std::string input_keyexpr = VIDEO_INPUT_KEYEXPR;
    app.add_option("-i,--input-key", input_keyexpr, "The key expression to subscribe video from")
        ->default_val(VIDEO_INPUT_KEYEXPR);
    std::string output_keyexpr = VIDEO_OUTPUT_KEYEXPR;
    app.add_option("-o,--output-key", output_keyexpr, "The key expression to publish the result to")
        ->default_val(VIDEO_OUTPUT_KEYEXPR);
    std::string backend = DEFAULT_BACKEND;
    app.add_option("-b,--backend", backend, "Inference backend to use (onnxruntime or tensorrt)")
        ->default_val(DEFAULT_BACKEND);
    std::string precision = DEFAULT_PRECISION;
    app.add_option("-p,--precision", precision, "Precision for the backend (cpu, cuda, fp16, fp32)")
        ->default_val(DEFAULT_PRECISION);
    int gpu_id = DEFAULT_GPU_ID;
    app.add_option("-g,--gpu-id", gpu_id, "GPU ID to use for CUDA backend")
        ->default_val(DEFAULT_GPU_ID);
    std::string model_type = "scene";
    app.add_option("-m,--model-type", model_type, "Type of the model (segmentation, domain, or egolanes)")
        ->default_val("segmentation");
    CLI11_PARSE(app, argc, argv);

    try {
        // Initialize the segmentation engine
        std::unique_ptr<InferenceBackend> backend_;
        if (backend == "onnxruntime") {
            backend_ = std::make_unique<OnnxRuntimeBackend>(model_path, precision, gpu_id);
        } else if (backend == "tensorrt") {
            backend_ = std::make_unique<TensorRTBackend>(model_path, precision, gpu_id);
        } else {
            throw std::invalid_argument("Unknown backend type.");
        }

        // Zenoh Initialization
        // Create Zenoh session
        z_owned_config_t config;
        z_owned_session_t s;
        z_config_default(&config);
        if (z_open(&s, z_move(config), NULL) < 0) {
            throw std::runtime_error("Error opening Zenoh session");
        }
        // Declare a Zenoh subscriber
        z_owned_subscriber_t sub;
        z_view_keyexpr_t in_ke;
        z_view_keyexpr_from_str(&in_ke, input_keyexpr.c_str());
        z_owned_ring_handler_sample_t handler;
        z_owned_closure_sample_t closure;
        z_ring_channel_sample_new(&closure, &handler, RECV_BUFFER_SIZE);
        if (z_declare_subscriber(z_loan(s), &sub, z_loan(in_ke), z_move(closure), NULL) < 0) {
            throw std::runtime_error("Error declaring Zenoh subscriber for key expression: " + input_keyexpr);
        }
        // Declare a Zenoh publisher for the output
        z_owned_publisher_t pub;
        z_view_keyexpr_t out_ke;
        z_view_keyexpr_from_str(&out_ke, output_keyexpr.c_str());
        if (z_declare_publisher(z_loan(s), &pub, z_loan(out_ke), NULL) < 0) {
            throw std::runtime_error("Error declaring Zenoh publisher for key expression: " + output_keyexpr);
        }

        // Subscribe to the input key expression and process frames
        std::cout << "Subscribing to '" << input_keyexpr << "'..." << std::endl;
        std::cout << "Publishing results to '" << output_keyexpr << "'..." << std::endl;
        z_owned_sample_t sample;

        // Benchmark: Output results at a certain frequency
        FpsTimer timer(BENCHMARK_OUTPUT_FREQUENCY);

        while (Z_OK == z_recv(z_loan(handler), &sample)) {
            // Benchmark: Receive new frame
            timer.startNewFrame();

            // Get the loaned sample and extract the payload
            Mat frame;
            try {
                frame = decode_frame_from_sample(sample);
            } catch (const std::exception& e) {
                std::cerr << "Decode error: " << e.what() << std::endl;
                z_drop(z_move(sample));
                continue;
            }

            // Benchmark: Preprocess done
            timer.recordPreprocessEnd();

            // Run inference
            if (!backend_->doInference(frame)) {
                z_drop(z_move(sample));
                throw std::runtime_error("Failed to run inference on the frame");
            }

            // Model-type specific processing
            const float* tensor_data = backend_->getRawTensorData();
            std::vector<int64_t> tensor_shape = backend_->getTensorShape();
            if (tensor_shape.size() != 4) {
                z_drop(z_move(sample));
                throw std::runtime_error("Invalid tensor shape");
            }
            cv::Mat final_frame;
            if (model_type == "depth") {
                // Depth estimation: output raw depth values (CV_32FC1)
                int height = static_cast<int>(tensor_shape[2]);
                int width = static_cast<int>(tensor_shape[3]);
              
                // Create depth map from tensor data (single channel float)
                cv::Mat depth_map(height, width, CV_32FC1, const_cast<float*>(tensor_data));
              
                // Resize depth map to original image size (use LINEAR for depth)
                cv::Mat resized_depth;
                cv::resize(depth_map, resized_depth, frame.size(), 0, 0, cv::INTER_LINEAR);
              
                //// Only send out the depth
                final_frame = resized_depth;
                //// Debug: Show the blended result directly
                // std::unique_ptr<autoware_pov::common::DepthVisualizationEngine> viz_engine_ = 
                //     std::make_unique<autoware_pov::common::DepthVisualizationEngine>();
                // final_frame = viz_engine_->visualize(resized_depth);
                
            } else if (model_type == "segmentation" || model_type == "egolanes") {
                cv::Mat mask;
 
#ifdef CUDA_FOUND
                bool cuda_success = false;
                // Try CUDA acceleration first
                if (model_type == "segmentation") {
                    cuda_success = autoware_pov::common::MasksVisualizationKernels::createMaskFromTensorCUDA(
                        tensor_data, tensor_shape, mask
                    );
                }
                else if (model_type == "egolanes") {
                    cuda_success = autoware_pov::common::MasksVisualizationKernels::createEgoLanesMaskFromTensorCUDA(
                        tensor_data, tensor_shape, mask
                    );
                }

                if (!cuda_success)
#endif
                {
                    // CPU fallback: create mask from tensor
                    int height = static_cast<int>(tensor_shape[2]);
                    int width = static_cast<int>(tensor_shape[3]);
                    int channels = static_cast<int>(tensor_shape[1]);
      
                    mask = cv::Mat(height, width, CV_8UC1);
      
                    if (channels > 1) {
			            if (model_type == "segmentation"){
                       	    // Multi-class segmentation: argmax across channels (NCHW format)
                            for (int h = 0; h < height; ++h) {
                                for (int w = 0; w < width; ++w) {
                                    float max_score = -1e9f;
                                    uint8_t best_class = 0;
                                    for (int c = 0; c < channels; ++c) {
                                        // NCHW format: tensor_data[batch=0][channel=c][height=h][width=w]
                                        float score = tensor_data[c * height * width + h * width + w];
                                        if (score > max_score) {
                                            max_score = score;
                                            best_class = static_cast<uint8_t>(c);
                                        }
                                    }
                                    // Convert class IDs for scene segmentation: Class 1 -> 255, others -> 0
                                    mask.at<uint8_t>(h, w) = (best_class == 1) ? 255 : 0;
                                }
                            }
                        } else if (model_type == "egolanes") {
                            const int HW = height * width;

                            const float* c0 = tensor_data + 0 * HW; // ego-left
                            const float* c1 = tensor_data + 1 * HW; // ego-right
                            const float* c2 = tensor_data + 2 * HW; // all_other_lanes

                            for (int h = 0; h < height; ++h) {
                                for (int w = 0; w < width; ++w) {
                                    const int idx = h * width + w;

                                    const bool b0 = (c0[idx] > 0.0f);
                                    const bool b1 = (c1[idx] > 0.0f);
                                    const bool b2 = (c2[idx] > 0.0f);

                                    if (b2) {
                                        mask.at<uint8_t>(h, w) = 2;
                                    } else if (b1) {
                                        mask.at<uint8_t>(h, w) = 1;
                                    } else if (b0) {
                                        mask.at<uint8_t>(h, w) = 0;
                                    } else {
                                        mask.at<uint8_t>(h, w) = 255; // background
                                    }
                                }
                            }
                        }
                    } else {
                        // Single channel: threshold for binary segmentation
                        for (int h = 0; h < height; ++h) {
                            for (int w = 0; w < width; ++w) {
                                float value = tensor_data[h * width + w];
                                mask.at<uint8_t>(h, w) = (value > 0.0f) ? 255 : 0;
                            }
                        }
                    }
                }

                // Resize mask to original image size (use NEAREST for masks)
                cv::Mat resized_mask;
                cv::resize(mask, resized_mask, frame.size(), 0, 0, cv::INTER_NEAREST);

                //// Only send out the mask
                final_frame = resized_mask;
                //// Debug: Show the blended result directly
                //std::unique_ptr<autoware_pov::common::MasksVisualizationEngine> viz_engine_ = 
                //    std::make_unique<autoware_pov::common::MasksVisualizationEngine>("scene");
                //final_frame = viz_engine_->visualize(resized_mask, frame);
            }

            // Benchmark: Inference done
            timer.recordInferenceEnd();

            // Publish the processed frame via Zenoh
            z_publisher_put_options_t options;
            z_publisher_put_options_default(&options);
            // Create attachment with frame metadata
            z_owned_bytes_t attachment_out;
            int output_bytes_info[] = {final_frame.rows, final_frame.cols, final_frame.type()};
            z_bytes_copy_from_buf(&attachment_out, (const uint8_t*)output_bytes_info, sizeof(output_bytes_info));
            options.attachment = z_move(attachment_out);
            // Create payload with pixel data and publish
            unsigned char* pixelPtr = final_frame.data;
            size_t dataSize = final_frame.total() * final_frame.elemSize();
            z_owned_bytes_t payload_out;
            z_bytes_copy_from_buf(&payload_out, pixelPtr, dataSize);
            z_publisher_put(z_loan(pub), z_move(payload_out), &options);
                
            // Benchmark: Output done
            timer.recordOutputEnd();

            // Release sample
            z_drop(z_move(sample));
        }
        
        // Cleanup
        z_drop(z_move(pub));
        z_drop(z_move(handler));
        z_drop(z_move(sub));
        z_drop(z_move(s));
	    cv::destroyAllWindows();
    } catch (const std::exception& e) {
        std::cerr << "Standard error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
