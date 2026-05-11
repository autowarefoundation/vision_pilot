#ifndef VISIONPILOT_V4L2_READER_HPP
#define VISIONPILOT_V4L2_READER_HPP

#include <opencv2/opencv.hpp>
#include <cstdint>
#include <mutex>
#include <tuple>
#include <string>
#include <memory>
#include <iostream>

namespace v4l2_interface {

    class V4L2Reader {

        public:

        /**
        * @brief Constructor for V4L2Reader
        *
        * @param device_path Mounting path to V4L2 device (e.g., "/dev/video0")
        @ @param fps Desired FPS for outputing frames. Current default is 10.
        * 
        * Inits the V4L2 camera device with specified parameters and start capture/vis thread.
        * Also logs these details and subscribtion info.
        */    
        explicit V4L2Reader(
            const std::string& device_path,
            uint32_t fps = 10
        );

        /**
        * @brief Destructor for V4L2Reader
        *
        * Properly cleans up camera resources and closes video capture threads.
        */
        ~V4L2Reader();

    }

}

#endif //VISIONPILOT_V4L2_READER_HPP
