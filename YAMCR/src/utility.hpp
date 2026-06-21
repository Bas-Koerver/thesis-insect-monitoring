#ifndef YAMCR_SRC_UTILITY_HPP
#define YAMCR_SRC_UTILITY_HPP
#include <opencv2/imgproc.hpp>

namespace YAMCR::Utility {
    void saveMatToDisk(const cv::Mat& frame, const std::filesystem::path& path);
} // YAMCR::Utility

#endif //YAMCR_SRC_UTILITY_HPP
