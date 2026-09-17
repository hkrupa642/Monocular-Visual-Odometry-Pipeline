cmake_minimum_required(VERSION 3.16)

project(MonocularVO)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    src
)

add_executable(MonocularVO
    src/main.cpp
    src/FeatureDetector.cpp
    src/MotionEstimator.cpp
    src/Pose.cpp
)

target_link_libraries(MonocularVO
    ${OpenCV_LIBS}
    Eigen3::Eigen
)
