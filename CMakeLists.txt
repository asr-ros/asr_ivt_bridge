cmake_minimum_required(VERSION 2.8.3)
project(asr_ivt_bridge)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    ivt
    image_geometry
)

catkin_package(
	INCLUDE_DIRS	include
	LIBRARIES	${PROJECT_NAME}
	CATKIN_DEPENDS roscpp sensor_msgs ivt image_geometry
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
	src/ivt_image.cpp 
	src/ivt_calibration.cpp
)

target_link_libraries (${PROJECT_NAME} 
	${catkin_LIBRARIES}
)
