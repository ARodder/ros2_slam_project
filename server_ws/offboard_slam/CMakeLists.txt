cmake_minimum_required(VERSION 3.8)
project(offboard_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/save_slam_session.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
