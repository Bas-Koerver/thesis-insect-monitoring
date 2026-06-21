#include "utility.hpp"

#include <filesystem>

#include <opencv2/imgcodecs.hpp>


namespace YAMCR::Utility {
    void saveMatToDisk(const cv::Mat& frame, const std::filesystem::path& path) {
        (void)cv::imwrite(path.string(), frame);
    }
} // YAMCR::Utility

