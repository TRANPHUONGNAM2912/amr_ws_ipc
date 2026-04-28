cmake_minimum_required(VERSION 3.8)
project(agv_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config web
  DESTINATION share/${PROJECT_NAME}
)

# Install Python teleop script as an executable in lib/<pkg>
install(
  PROGRAMS scripts/teleop_panel.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

