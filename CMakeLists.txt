cmake_minimum_required(VERSION 3.0.0)
project(robot VERSION 0.1.0)

include(CTest)
include_directories("${Project_Source_Dir}/Arm")
include_directories("${Project_Source_Dir}/serial")
enable_testing()

add_subdirectory(Arm)
add_subdirectory(serial)
add_executable(robot main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
