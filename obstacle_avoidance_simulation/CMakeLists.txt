cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance_simulation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

add_message_files(
  FILES
  ConfigGenes.msg
  RobotInfo.msg
  ControlSimulation.msg
  EvolutiveSystemConfiguration.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs message_runtime gazebo_ros std_srvs gazebo_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(robot_obstacle_avoidance_node src/robot_obst_avoid.cpp)
target_link_libraries(robot_obstacle_avoidance_node ${catkin_LIBRARIES})
#add_dependencies(robot_obstacle_avoidance_node beginner_tutorials_gencpp)
