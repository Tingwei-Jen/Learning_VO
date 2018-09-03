cmake_minimum_required( VERSION 2.8 )
project( LearningVO )

set( CMAKE_CXX_COMPILER "g++" )
#set( CMAKE_BUILD_TYPE "Release" )
#set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(SOURCE_FILES 
	src/main.cpp
	src/pinhole_camera.cpp
	src/visual_odometry.cpp
)

add_executable(${PROJECT_NAME}  ${SOURCE_FILES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
