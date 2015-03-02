cmake_minimum_required(VERSION 2.8.3)
project(evaluate_omni_dataset)

find_package(cmake_modules REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  tf 
  geometry_msgs     
  nav_msgs
  rosbag
  cv_bridge
  message_generation
  image_transport
  read_omni_dataset
)

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RobotState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs     
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(evaluate_omni_dataset src/evaluate_omni_dataset.cpp)
target_link_libraries(evaluate_omni_dataset ${catkin_LIBRARIES} ${rosbag_LIBRARIES})