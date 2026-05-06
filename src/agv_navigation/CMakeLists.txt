cmake_minimum_required(VERSION 3.8)
project(agv_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
    maps
    rviz
    params
    behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/route_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
