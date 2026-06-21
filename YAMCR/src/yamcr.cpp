#include <opencv2/opencv.hpp>

#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>

#include <string>
#include <vector>

#include "post_processors/basler_image_converter.hpp"
#include "post_processors/prophesee_event_extractor.hpp"


int main() {
    const std::vector<std::filesystem::path> inputDirs = {
        // "D:/datasets/own_data/mason_bees_1/raw/cam_1",
        // "D:/datasets/own_data/mason_bees_2/raw/cam_1",
        // "D:/datasets/own_data/mason_bees_3/raw/cam_1",
        // "D:/datasets/own_data/mason_bees_4/raw/cam_1",
        // "D:/datasets/own_data/mason_bees_5/raw/cam_1",
        // "D:/datasets/own_data/mason_bees_6/raw/cam_1",
        "C:/Users/Bas_K/OneDrive/Documenten/School/Maastricht university/wafer_research/datasets/silicon_test_3/raw/cam_1"
    };
    YAMCR::PostProcessors::imageParams params {
        .width=1920,
        .height=1200,
        .pngCompression=6,
        .maxWorkers = 9
    };

    YAMCR::PostProcessors::processBaslerData(inputDirs, params);

    // for (auto inputDir : inputDirs) {
    //     YAMCR::PostProcessors::processPropheseeData(inputDir.parent_path() / "cam_0");
    // }

    return 0;
}
