#include "prophesee_event_extractor.hpp"

#include "../utility.hpp"

#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <metavision/sdk/stream/camera.h>


namespace YAMCR::PostProcessors {

    void processPropheseeData(std::filesystem::path rawPath) {
        std::filesystem::path eventFile{rawPath / "event_file.hdf5"};

        Metavision::Camera cam;
        try {
            cam = Metavision::Camera::from_file(eventFile);
        } catch (Metavision::CameraException& e) {
            std::cerr << e.what() << "\n";
        }

        const Metavision::I_Geometry& geometry = cam.geometry();
        auto frameIndex{0};
        auto fallingEdgePolarity{1};
        Metavision::timestamp eventStartTime{0};
        std::filesystem::path processedPath{rawPath.parent_path().parent_path() / "processed" / rawPath.stem() };

        (void)std::filesystem::create_directories(processedPath);

        Metavision::OnDemandFrameGenerationAlgorithm onDemandFrameGenerator{
            geometry.get_width(),
            geometry.get_height()
        };

        (void)cam.ext_trigger().add_callback(
                [&onDemandFrameGenerator, &frameIndex, &fallingEdgePolarity, &eventStartTime, &processedPath](
                const Metavision::EventExtTrigger* begin,
                const Metavision::EventExtTrigger* end) {
                    for (auto ev = begin; ev != end; ++ev) {
                        if (ev->p == !fallingEdgePolarity) {
                            eventStartTime = ev->t;
                        }
                        if (ev->p == fallingEdgePolarity) {
                            ++frameIndex;
                            cv::Mat frame;
                            onDemandFrameGenerator.set_accumulation_time_us(ev->t - eventStartTime);
                            onDemandFrameGenerator.generate(ev->t, frame);

                            std::filesystem::path imagePath{processedPath / ("frame_" + std::to_string(frameIndex) + ".png")};
                            Utility::saveMatToDisk(frame, imagePath);
                        }
                    }


                });

        (void)cam.cd().add_callback(
                [&onDemandFrameGenerator](
                const Metavision::EventCD* begin,
                const Metavision::EventCD* end) {

                    onDemandFrameGenerator.process_events(begin, end);
                });

        cam.start();

        while (cam.is_running()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
} // YAMCR