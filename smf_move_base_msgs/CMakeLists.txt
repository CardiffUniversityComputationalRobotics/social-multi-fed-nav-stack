cmake_minimum_required(VERSION 3.0.2)
project(smf_move_base_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pedsim_msgs
  message_generation
)

add_message_files(
  FILES
  RelevantAgentState.msg
  RelevantAgentStates.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  pedsim_msgs
  smf_move_base_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES smf_move_base_msgs
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


