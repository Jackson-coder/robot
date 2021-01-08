add_library(arm Arm.cpp)

set(OpenCV_DIR /home/opencv/build)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries( arm ${OpenCV_LIBS} )
target_link_libraries(arm serial)

