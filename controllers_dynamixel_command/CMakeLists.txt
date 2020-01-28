cmake_minimum_required(VERSION 2.8.3)
project(controllers_dynamixel_command)

find_package(
  catkin REQUIRED COMPONENTS 
  std_msgs 
  roscpp
  dynamixel_workbench_msgs
  )

catkin_package(
  LIBRARIES controllers_dynamixel_command
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(controllers_dynamixel_command src/controllers_dynamixel_command.cpp)
add_dependencies(controllers_dynamixel_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controllers_dynamixel_command ${catkin_LIBRARIES})