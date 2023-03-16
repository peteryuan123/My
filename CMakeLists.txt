cmake_minimum_required(VERSION 3.1)

project(MySLAM VERSION 1.0)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)


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

add_executable(test_sim3 test/test_sim3.cpp)
target_link_libraries(test_sim3
        g2o_core
        g2o_stuff
        g2o_types_sim3
        MySLAM
        stdc++fs
        )

add_executable(test_se3 test/test_se3.cpp)
target_link_libraries(test_se3
        g2o_core
        g2o_stuff
        g2o_types_sim3
        g2o_types_slam3d
        g2o_types_sba
        MySLAM
        stdc++fs
        )

add_executable(test_ceres_sim3 test/test_ceres_sim3.cpp)
target_link_libraries(test_ceres_sim3
        ${CERES_LIBRARIES}
        MySLAM
        stdc++fs
        )