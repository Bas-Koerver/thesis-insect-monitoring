#ifndef YAMCR_SRC_POST_PROCESSORS_BASLER_IMAGE_CONVERTER_HPP
#define YAMCR_SRC_POST_PROCESSORS_BASLER_IMAGE_CONVERTER_HPP
#include <filesystem>

namespace YAMCR::PostProcessors {
    struct imageParams {
        int width;
        int height;
        int pngCompression;
        int maxWorkers;
    };


    int processBaslerData(std::vector<std::filesystem::path> inputDirs, imageParams params);
} // YAMCR::PostProcessors

#endif //YAMCR_SRC_POST_PROCESSORS_BASLER_IMAGE_CONVERTER_HPP
