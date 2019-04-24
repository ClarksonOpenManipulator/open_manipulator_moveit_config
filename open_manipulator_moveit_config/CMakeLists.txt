cmake_minimum_required(VERSION 2.8.12)

project(open_manipulator_moveit_config)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS moveit_ros_planning trajectory_msgs message_generation moveit_core moveit_ros_planning moveit_ros_planning_interface)

add_service_files(
  FILES
  MoveGroupCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS moveit_ros_planning trajectory_msgs moveit_ros_planning_interface moveit_core trajectory_msgs 
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/cubic_spline_adapter.cpp 
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(commander_service src/move_group_commander_service.cpp)
add_dependencies(commander_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(commander_service ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES planning_request_adapters_plugin_description.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
