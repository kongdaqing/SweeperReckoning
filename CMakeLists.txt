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


add_executable(eurocAttitudeEstimation src/eurocAttEst.cpp
    src/Euroc/readEurocData.cpp
    src/Fusion/mahonyAttEstimator.cpp
    src/Sensor/imu.cpp
    src/Sensor/odometry.cpp
    src/Sensor/optFlow.cpp
    src/Utility/lowPassFilter.cpp)
target_link_libraries(eurocAttitudeEstimation  ${OpenCV_LIBS})

add_executable(simAttitudeEstimation src/simAttEst.cpp
    src/Fusion/mahonyAttEstimator.cpp
    src/Sensor/imu.cpp
    src/Sensor/odometry.cpp
    src/Sensor/optFlow.cpp
    src/Utility/lowPassFilter.cpp)
target_link_libraries(simAttitudeEstimation  ${OpenCV_LIBS})

add_executable(compensateOdomByOpt src/testCompensate.cpp
    src/Fusion/mahonyAttEstimator.cpp
    src/Reckoning/DeadReckoning.cpp
    src/AnomalyDetector/anomalyDetector.cpp
    src/Sensor/imu.cpp
    src/Sensor/odometry.cpp
    src/Sensor/optFlow.cpp
    src/Utility/lowPassFilter.cpp
    src/Anker/readAnkerDataFile.cpp)

target_link_libraries(compensateOdomByOpt  ${OpenCV_LIBS})


add_library(ankerSweeperLib src/testCompensate.cpp
    src/Fusion/mahonyAttEstimator.cpp
    src/Reckoning/DeadReckoning.cpp
    src/AnomalyDetector/anomalyDetector.cpp
    src/Sensor/imu.cpp
    src/Sensor/odometry.cpp
    src/Sensor/optFlow.cpp
    src/Utility/lowPassFilter.cpp
    src/Anker/readAnkerDataFile.cpp)
target_link_libraries(ankerSweeperLib  ${OpenCV_LIBS})
