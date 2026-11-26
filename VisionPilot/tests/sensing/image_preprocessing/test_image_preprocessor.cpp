#include <iostream>
#include <ostream>
#include <string>
#include <image_preprocessing/image_preprocessor.hpp>


int main(const int argc, const char *argv[]) {
    cv::Mat warped_image;
    cv::Mat resized_image;
    const auto size = cv::Size(1024, 512);
    ImagePreprocessor image_preprocessor;

    if (argc < 2) {
        std::cerr << "Usage: test <image_path>" << std::endl;
        return -1;
    }

    const std::string image_path = argv[1];
    const cv::Mat image = cv::imread(image_path);

    if (image.empty()) {
        std::cerr << "Failed to load image!" << std::endl;
        return -1;
    }

    std::cout << "Image loaded: " << image.cols << "x" << image.rows << std::endl;

    image_preprocessor.preprocess(image, warped_image, resized_image, size);
    std::cout << "Image warped: " << warped_image.cols << "x" << warped_image.rows << std::endl;

#ifdef VISIONPILOT_ENABLE_GUI
    cv::namedWindow("Warped Image", cv::WINDOW_NORMAL);
    cv::imshow("Warped Image", warped_image);
    cv::imshow("Resized image", resized_image);
    cv::resizeWindow("Resized image", size.width, size.height);
    cv::waitKey(0); // waits for key press
#endif

    return 0;
}
