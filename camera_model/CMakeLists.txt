cmake_minimum_required(VERSION 2.8.3)
project(camera_model)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    )

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES camera_model
    CATKIN_DEPENDS roscpp std_msgs
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

include_directories("include")

add_library(camera_model
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/PinholeCamera.cc)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS})
