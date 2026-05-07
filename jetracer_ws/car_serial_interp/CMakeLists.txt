cmake_minimum_required(VERSION 3.8)
project(car_serial_interp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

add_executable(jetracer_node src/jetracer.cpp)
add_executable(custom_ekf_node src/custom_ekf_node.cpp)
add_executable(odom_csv_logger_node src/odom_csv_logger_node.cpp)

ament_target_dependencies(jetracer_node
        rclcpp
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
)

ament_target_dependencies(custom_ekf_node
        rclcpp
        geometry_msgs
        nav_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
)

ament_target_dependencies(odom_csv_logger_node
        rclcpp
        nav_msgs
        tf2
)

target_compile_features(jetracer_node PUBLIC cxx_std_17)
target_compile_features(custom_ekf_node PUBLIC cxx_std_17)
target_compile_features(odom_csv_logger_node PUBLIC cxx_std_17)

install(TARGETS
        jetracer_node
        custom_ekf_node
        odom_csv_logger_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
