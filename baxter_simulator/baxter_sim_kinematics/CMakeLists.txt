cmake_minimum_required(VERSION 2.8.3)
project(baxter_sim_kinematics)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  baxter_core_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  kdl_parser
  gazebo_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  std_msgs
  baxter_core_msgs
  gazebo_msgs
  sensor_msgs
  kdl_parser
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/arm_kinematics.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(kinematics src/position_kinematics.cpp)
target_link_libraries(kinematics ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(kinematics ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_kinematics test/test_kinematics.cpp)
  if(TARGET test_kinematics)
    target_link_libraries(test_kinematics ${catkin_LIBRARIES} ${PROJECT_NAME})
    add_dependencies(test_kinematics ${catkin_EXPORTED_TARGETS})
  endif()
endif()

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS kinematics
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
