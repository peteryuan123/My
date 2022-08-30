cmake_minimum_required(VERSION 3.1)

project(MySLAM VERSION 1.0)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

file(GLOB Camera_SRC
        "src/Camera/*.cpp"
        "include/Camera/*.h"
        )

file(GLOB Tool_SRC
        "src/Tool/*.cpp"
        "include/Tool/*.h")

file(GLOB Frame_SRC
        "src/Frame/*.cpp"
        "include/Frame/*.h")

add_library(MySLAM SHARED ${Camera_SRC} ${Frame_SRC} ${Tool_SRC})
target_link_libraries(MySLAM
        ${OpenCV_LIBRARIES}
        )

add_executable(test_camera test/test_camera.cpp)
target_link_libraries(test_camera
        MySLAM
        stdc++fs
        )

