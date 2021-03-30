cmake_minimum_required(VERSION 3.5)
project(orbslam2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

set(LIBS 
-lfbow
-lDLib
-lg2o
-lORB_SLAM2
-lboost_system
)

add_executable(orbslam2 src/orbslam2.cpp)
ament_target_dependencies(orbslam2 rclcpp std_msgs sensor_msgs realsense2_camera_msgs realsense2_description realsense2_camera Eigen3 Pangolin)

target_link_libraries(orbslam2 ${LIBS})

install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
    )

install(TARGETS
  orbslam2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
