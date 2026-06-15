#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <chrono>
#include <cstring>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <CLI/CLI.hpp>
#include <zenoh.h>

using namespace cv;
using namespace std;

#define DEFAULT_KEYEXPR "video/raw"

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

int main(int argc, char** argv) {
    // Parse command line arguments
    CLI::App app{"Carla ROS2 Image subscriber example"};
    // Add options
    std::string keyexpr = DEFAULT_KEYEXPR;
    app.add_option("-k,--key", keyexpr, "The key expression to subscribe to")->default_val(DEFAULT_KEYEXPR);
    CLI11_PARSE(app, argc, argv);

    try {
        // Create Zenoh session
        z_owned_config_t config;
        z_owned_session_t s;
        z_config_default(&config);
        if (z_open(&s, z_move(config), NULL) < 0) {
            throw std::runtime_error("Error opening Zenoh session");
        }

        // Declare a Zenoh subscriber
        z_owned_subscriber_t sub;
        z_view_keyexpr_t ke;
        z_view_keyexpr_from_str(&ke, keyexpr.c_str());
        z_owned_ring_handler_sample_t handler;
        z_owned_closure_sample_t closure;
        z_ring_channel_sample_new(&closure, &handler, RECV_BUFFER_SIZE);
        if (z_declare_subscriber(z_loan(s), &sub, z_loan(ke), z_move(closure), NULL) < 0) {
            throw std::runtime_error("Error declaring Zenoh subscriber for key expression: " + keyexpr);
        }
        
        std::cout << "Subscribing to '" << keyexpr << "'..." << std::endl;
        std::cout << "Processing video... Press ESC to stop." << std::endl;
        z_owned_sample_t sample;
        while (Z_OK == z_recv(z_loan(handler), &sample)) {
            try {
                // Decode CDR -> cv::Mat
                cv::Mat frame = decode_frame_from_sample(sample);
                cv::imshow("Play video", frame);
                if (cv::waitKey(1) == 27) { // Stop if 'ESC' is pressed
                    std::cout << "Processing stopped by user." << std::endl;
                    z_drop(z_move(sample));
                    break;
                }

                // Print frame rate
                static int frame_count = 0;
                static auto start_time = std::chrono::steady_clock::now();
                frame_count++;
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
                if (elapsed_time > 0) {
                    double fps = static_cast<double>(frame_count) / elapsed_time;
                    std::cout << "Current FPS: " << fps << std::endl;
                    frame_count = 0;
                    start_time = current_time;
                }

            } catch (const std::exception& e) {
                std::cerr << "Decode error: " << e.what() << std::endl;
            }

            // Release sample
            z_drop(z_move(sample));
        }

        // Clean up
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
