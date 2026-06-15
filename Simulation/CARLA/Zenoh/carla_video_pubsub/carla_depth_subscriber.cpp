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

#define DEFAULT_KEYEXPR "video/output"
#define RECV_BUFFER_SIZE 100

cv::Mat decode_depth_from_sample(const z_owned_sample_t& sample) {
    const z_loaned_sample_t* loaned_sample = z_loan(sample);
    z_owned_slice_t zslice;
    if (Z_OK != z_bytes_to_slice(z_sample_payload(loaned_sample), &zslice)) {
        throw std::runtime_error("Wrong payload");
    }
    
    const uint8_t* ptr = z_slice_data(z_loan(zslice));
    size_t len = z_slice_len(z_loan(zslice));

    if (!ptr || len == 0) {
        z_drop(z_move(zslice));
        throw std::runtime_error("Empty depth payload");
    }

    // Read attachment: {rows, cols, type}
    const z_loaned_bytes_t* attachment = z_sample_attachment(loaned_sample);
    if (attachment == NULL) {
        z_drop(z_move(zslice));
        throw std::runtime_error("Depth sample has no attachment");
    }

    z_owned_slice_t att_slice;
    if (Z_OK != z_bytes_to_slice(attachment, &att_slice)) {
        z_drop(z_move(zslice));
        throw std::runtime_error("Failed to convert depth attachment to slice");
    }

    if (z_slice_len(z_loan(att_slice)) < 3 * sizeof(int)) {
        z_drop(z_move(zslice));
        z_drop(z_move(att_slice));
        throw std::runtime_error("Depth attachment too small");
    }

    int meta[3];
    std::memcpy(meta, z_slice_data(z_loan(att_slice)), 3 * sizeof(int));
    z_drop(z_move(att_slice));

    int rows = meta[0];
    int cols = meta[1];
    int type = meta[2];

    if (rows <= 0 || cols <= 0) {
        z_drop(z_move(zslice));
        throw std::runtime_error("Invalid depth dimensions");
    }

    cv::Mat depth_frame(rows, cols, type);
    size_t expected_size = static_cast<size_t>(rows) * cols * depth_frame.elemSize();
    if (len < expected_size) {
        z_drop(z_move(zslice));
        throw std::runtime_error("Depth payload size mismatch");
    }

    std::memcpy(depth_frame.data, ptr, expected_size);
    z_drop(z_move(zslice));

    return depth_frame;
}

int main(int argc, char** argv) {
    // Parse command line arguments
    CLI::App app{"Carla depth subscriber example"};
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
            throw std::runtime_error("Error declaring Zenoh subscriber for key: " + keyexpr);
        }

        std::cout << "Subscribing to '" << keyexpr << "'..." << std::endl;
        std::cout << "Processing depth stream... Press ESC to stop." << std::endl;

        z_owned_sample_t sample;
        while (Z_OK == z_recv(z_loan(handler), &sample)) {
            try {
                cv::Mat depth_frame = decode_depth_from_sample(sample);
                z_drop(z_move(sample));

                // Normalize for visualization
                cv::Mat depth_vis;
                cv::normalize(depth_frame, depth_vis, 0, 255, cv::NORM_MINMAX);
                depth_vis.convertTo(depth_vis, CV_8UC1);
                cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);

                cv::imshow("Depth", depth_vis);
                if (cv::waitKey(1) == 27) {
                    std::cout << "Processing stopped by user." << std::endl;
                    break;
                }

                // Print frame rate
                static int frame_count = 0;
                static auto start_time = std::chrono::steady_clock::now();
                frame_count++;
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_time =
                    std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time)
                        .count();
                if (elapsed_time > 0) {
                    double fps = static_cast<double>(frame_count) / elapsed_time;
                    std::cout << "Current FPS: " << fps << std::endl;
                    frame_count = 0;
                    start_time = current_time;
                }

            } catch (const std::exception& e) {
                std::cerr << "Decode error: " << e.what() << std::endl;
                z_drop(z_move(sample));
            }
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
