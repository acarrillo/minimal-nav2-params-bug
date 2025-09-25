cmake_minimum_required(VERSION 3.5)
project(nav2_bug_repro_pkg)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  maps
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/path_request_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()