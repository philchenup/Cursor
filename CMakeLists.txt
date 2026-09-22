cmake_minimum_required(VERSION 3.16)
project(white_region_sampler LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_library(white_region_sampler
    src/WhiteRegionSampler.cpp
)
target_include_directories(white_region_sampler PUBLIC include)
target_link_libraries(white_region_sampler PUBLIC ${OpenCV_LIBS})

add_executable(white_region_sampler_example
    src/WhiteRegionSamplerExample.cpp
)
target_link_libraries(white_region_sampler_example PRIVATE white_region_sampler)
