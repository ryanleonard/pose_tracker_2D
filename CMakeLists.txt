cmake_minimum_required(VERSION 3.14)
project(pose_tracker_2D)

set(CMAKE_CXX_STANDARD 14)

add_executable(pose_tracker_2D main.cpp pose_tracker.cpp pose_tracker.h utils.cpp utils.h depth_processing.cpp depth_processing.h)

#==============
# Librealsense
#==============
find_package( realsense2 REQUIRED )
#find_package()
if( realsense2_FOUND )
    message("Found Realsense!")
    # Additional Include Directories
    include_directories( ${realsense_INCLUDE_DIR} )
    # Additional Dependencies
    target_link_libraries( pose_tracker_2D ${realsense2_LIBRARY} pthread)
endif()


find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    target_include_directories(pose_tracker_2D PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(pose_tracker_2D ${OpenCV_LIBS})
endif()