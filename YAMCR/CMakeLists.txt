cmake_minimum_required(VERSION 3.23...3.27)
project(YAMCR
        LANGUAGES CXX
        VERSION 0.1)

set(CMAKE_CXX_STANDARD 23)

find_package(MetavisionSDK COMPONENTS core stream ui REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(YAMCR
        src/yamcr.cpp

        src/utility.cpp                                     src/utility.hpp

        src/post_processors/prophesee_event_extractor.cpp   src/post_processors/prophesee_event_extractor.hpp
        src/post_processors/basler_image_converter.cpp
        src/post_processors/basler_image_converter.hpp
)

target_link_libraries(
        YAMCR
        PRIVATE
        MetavisionSDK::core MetavisionSDK::stream MetavisionSDK::ui
)