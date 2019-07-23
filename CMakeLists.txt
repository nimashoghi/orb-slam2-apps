cmake_minimum_required(VERSION 2.8)
project(slam_app)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(fmt REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
)

add_executable(
    slam_app
        src/main.cpp
)
target_link_libraries(
    slam_app
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        libg2o.so
        libDBoW2.so
        libORB_SLAM2.so
        fmt::fmt
)
install(
    TARGETS slam_app
        RUNTIME DESTINATION bin
)

add_executable(
    slam_app_live
        src/live.cpp
)
target_link_libraries(
    slam_app_live
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        libg2o.so
        libDBoW2.so
        libORB_SLAM2.so
        fmt::fmt
)
install(
    TARGETS slam_app_live
        RUNTIME DESTINATION bin
)
