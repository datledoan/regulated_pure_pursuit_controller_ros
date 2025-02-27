cmake_minimum_required(VERSION 3.0.2)
project(regulated_pure_pursuit_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  costmap_2d
  mbf_costmap_core
  mbf_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/RegulatedPurePursuit.cfg)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES regulated_pure_pursuit_controller
  CATKIN_DEPENDS
  angles
  base_local_planner
  costmap_2d
  mbf_costmap_core
  mbf_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
  dynamic_reconfigure
)

add_library(${PROJECT_NAME} SHARED
  src/regulated_pure_pursuit_controller.cpp
  src/collision_checker.cpp
  src/parameter_handler.cpp
  src/path_handler.cpp)
  
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  
  install(FILES regulated_pure_pursuit_controller.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  
  install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      PATTERN ".svn" EXCLUDE
  )