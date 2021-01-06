cmake_minimum_required(VERSION 3.0.0)
project(robot VERSION 0.1.0)

set(OpenCV_DIR /home/opencv/build)
# set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories("${Project_Source_Dir}/Arm")
include_directories("${Project_Source_Dir}/serial")
include_directories("${Project_Source_Dir}/camera")

add_subdirectory(Arm)
add_subdirectory(serial)
add_subdirectory(camera)
add_executable(robot main.cpp)

target_link_libraries( robot ${OpenCV_LIBS} )
target_link_libraries( robot arm)
target_link_libraries( robot camera)
target_link_libraries( robot serial)
