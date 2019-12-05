cmake_minimum_required(VERSION 2.8.3)
project(SweeperReckoning)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)
#find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)


include_directories()

add_executable(testReckoning src/test.cpp
    src/Fusion/mahonyAttEstimator.cpp
    src/Sensor/imu.cpp
    src/Sensor/odometry.cpp
    src/Utility/lowPassFilter.cpp)

target_link_libraries(testReckoning  ${OpenCV_LIBS})

add_executable(ankerReckoning src/AnkerInterface.cpp
    src/Fusion/mahonyAttEstimator.cpp
    src/Sensor/imu.cpp
    src/Sensor/odometry.cpp
    src/Utility/lowPassFilter.cpp
    src/Anker/readAnkerDataFile.cpp)

target_link_libraries(ankerReckoning  ${OpenCV_LIBS})
