cmake_minimum_required(VERSION 2.8.3)
project(exercise4)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	visualization_msgs
	tf2_ros
	nav_msgs
	std_msgs
	geometry_msgs
	message_generation
)

find_package( OpenCV REQUIRED )

# catkin_package()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Coords.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#   CATKIN_DEPENDS message_runtime ...
)

#include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_executable(map_goals src/map_goals.cpp)
#target_link_libraries(map_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#catkin_install_python(PROGRAMS scripts/detect_rings DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

