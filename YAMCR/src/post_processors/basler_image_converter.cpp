#include "basler_image_converter.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace {
    struct Job {
        std::filesystem::path inputPath;
        std::filesystem::path outputPath;
    };

    class JobQueue {
    public:
        void push(Job job) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                queue_.push(std::move(job));
            }
            cv_.notify_one();
        }


        std::optional<Job> pop() {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock,
                     [&] {
                         return done_ || !queue_.empty();
                     });

            if (queue_.empty()) {
                return std::nullopt;
            }

            Job job = std::move(queue_.front());
            queue_.pop();
            return job;
        }


        void set_done() {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                done_ = true;
            }
            cv_.notify_all();
        }


    private:
        std::queue<Job> queue_;
        std::mutex mutex_;
        std::condition_variable cv_;
        bool done_ = false;
    };


    static bool read_raw_file(const std::filesystem::path& path,
                              std::vector<std::uint8_t>& buffer,
                              std::size_t expectedSize) {
        std::ifstream file(path, std::ios::binary);
        if (!file) {
            return false;
        }

        buffer.resize(expectedSize);
        file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
        return file.good() || file.gcount() == static_cast<std::streamsize>(expectedSize);
    }


    static bool convert_one_file(
        const std::filesystem::path& inputPath,
        const std::filesystem::path& outputPath,
        int width,
        int height,
        int pngCompression
        ) {
        const std::size_t expectedSize = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

        std::vector<std::uint8_t> rawBuffer;
        if (!read_raw_file(inputPath, rawBuffer, expectedSize)) {
            std::cerr << "Failed to read: " << inputPath << '\n';
            return false;
        }

        cv::Mat raw(height, width, CV_8UC1, rawBuffer.data());

        cv::Mat bgr;
        cv::demosaicing(raw, bgr, cv::COLOR_BayerRGGB2BGR);

        const std::vector<int> params = {
            cv::IMWRITE_PNG_COMPRESSION,
            pngCompression
        };

        if (!cv::imwrite(outputPath.string(), bgr, params)) {
            std::cerr << "Failed to write: " << outputPath << '\n';
            return false;
        }

        return true;
    }
}


namespace YAMCR::PostProcessors {
    int processBaslerData(std::vector<std::filesystem::path> inputDirs, imageParams params) {
        JobQueue queue;
        std::atomic successCount = 0;
        std::atomic failCount = 0;

        std::vector<std::filesystem::path> inputFiles;
        for (const auto& inputDir : inputDirs) {
            std::filesystem::path outputDir{inputDir.parent_path().parent_path() / "processed" / inputDir.filename()};
            if (!is_directory(outputDir)) {
                (void)create_directories(outputDir);
            }
            for (const auto& inputPath : std::filesystem::directory_iterator(inputDir)) {
                if (inputPath.is_regular_file() && inputPath.path().extension() == ".raw") {
                    // inputFiles.push_back(entry.path());
                    const auto stem{inputPath.path().stem().string()};
                    const auto pos{stem.find_last_of('_')};
                    std::string paddedIndex{stem.substr(pos + 1)};
                    auto frameIndex{stoi(paddedIndex)};

                    ++frameIndex;

                    auto outputPath{outputDir / ("frame_" + std::to_string(frameIndex) + ".png")};
                    queue.push(Job{inputPath, outputPath});
                }
            }
        }



        unsigned int workerCount{std::thread::hardware_concurrency()};
        std::cout << workerCount << " concurrent threads are supported.\n";
        if (workerCount == 0) {
            workerCount = 4;
        }
        workerCount = std::min<unsigned int>(workerCount, params.maxWorkers);

        std::vector<std::jthread> workers;
        workers.reserve(workerCount);

        for (unsigned int i = 0; i < workerCount; ++i) {
            workers.emplace_back([&](std::stop_token) {
                while (true) {
                    std::optional<Job> job = queue.pop();
                    if (!job.has_value()) {
                        break;
                    }

                    const bool ok = convert_one_file(
                        job->inputPath,
                        job->outputPath,
                        params.width,
                        params.height,
                        params.pngCompression
                        );

                    if (ok) {
                        ++successCount;
                    } else {
                        ++failCount;
                    }
                }
            });
        }

        // for (const auto& inputPath : inputFiles) {
        //     std::filesystem::path outputPath = inputPath.parent_path().parent_path() / "processed" / inputPath.stem();
        //     outputPath += ".png";
        //     queue.push(Job{inputPath, outputPath});
        // }

        queue.set_done();

        // std::jthread joins automatically on destruction.
        workers.clear();

        std::cout << "Done. Success: " << successCount.load()
            << ", Failed: " << failCount.load() << '\n';

        return failCount.load() == 0 ? 0 : 1;

        // std::filesystem::path processedPath{rawPath.parent_path().parent_path() / "processed" / rawPath.stem()};
        // const std::vector<int> pngParams{cv::IMWRITE_PNG_COMPRESSION, 9};
        //
        // (void)std::filesystem::create_directories(processedPath);
        //
        // std::vector<std::filesystem::directory_entry> files;
        // for (auto file : std::filesystem::directory_iterator(rawPath)) {
        //     if (file.is_regular_file()) {
        //         files.push_back(std::move(file));
        //     }
        // }
        //
        // // Parallelise the conversion
        // std::for_each(std::execution::par, files.begin(), files.end(), [&processedPath, &pngParams](const auto& file) {
        //     const auto stem{file.path().stem().string()};
        //     const auto pos{stem.find_last_of('_')};
        //     std::string paddedIndex{stem.substr(pos + 1)};
        //     auto frameIndex{stoi(paddedIndex)};
        //
        //     ++frameIndex;
        //
        //     cv::Mat bayerImage = cv::imread(file.path().string(), cv::IMREAD_UNCHANGED);
        //     cv::Mat image;
        //     cv::cvtColor(bayerImage, image, cv::COLOR_BayerRGGB2BGR);
        //     if (image.empty()) {
        //         return;
        //     }
        //
        //     std::filesystem::path outPath{processedPath / ("frame_" + std::to_string(frameIndex) + ".png")};
        //
        //     (void)cv::imwrite(outPath.string(), image, pngParams);
        // });
    }

} // YAMCR::PostProcessors

