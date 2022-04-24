cmake_minimum_required(VERSION 3.5)

# Set default build type.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Default build type: Release" FORCE)
endif()

project(orbslam2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-implicit-fallthrough -Wno-maybe-uninitialized)
endif()

if(NOT HOST_COMPILATION)
  ADD_DEFINITIONS(-D__ANDROID__)
endif()

# Compilation options to set various features of the nodes.
# Activate them when building with colcon using:
# colcon build [...] --ament-cmake-args "-DOPTION=ON/OFF" "[...]" [...]
# Quote each option separately!!!
option(PX4 "Enable publishers of VIO data for PX4" OFF)
option(SMT "Enable multithreaded processing" ON)
option(DEBUG "Enable debug symbols and related compilation options" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
if(PX4)
  find_package(px4_msgs REQUIRED)
endif()

find_package(OpenCV 4.5.1 QUIET)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 4.5.1 not found.")
endif()
find_package(realsense2 REQUIRED)

if(NOT HOST_COMPILATION)
  find_package(CUDA REQUIRED)
  include_directories("${CUDA_INCLUDE_DIRS}"
                      /usr/local/include/ORB_SLAM2
                      ${PROJECT_SOURCE_DIR}/Drivers/RealSense
                      ${PROJECT_SOURCE_DIR}/include
                      ${EIGEN3_INCLUDE_DIR})
else()
  include_directories(/usr/local/include/ORB_SLAM2
  ${PROJECT_SOURCE_DIR}/Drivers/RealSense
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR})
endif()

# Activate debugging features.
if(DEBUG)
  message(STATUS "Activating debugging features")
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Debug build" FORCE)
endif()

# Check build type.
if(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "Release build type selected")
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  message(STATUS "Debug build type selected")
endif()

set(LIBS -lfbow
         -lDLib
         -lg2o
         -lORB_SLAM2
         -lboost_system)

add_executable(${PROJECT_NAME} src/fuser.cc src/pose.cc Drivers/RealSense/realsense.cc
                               src/fuser_ros2.cpp
                               src/fuser_node.cpp)

if(PX4)
  ament_target_dependencies(${PROJECT_NAME} rclcpp
                                            std_msgs
                                            sensor_msgs
                                            Eigen3
                                            Pangolin
                                            px4_msgs
                                            OpenCV
                                            realsense2)
else()
  ament_target_dependencies(${PROJECT_NAME} rclcpp
                                            std_msgs
                                            sensor_msgs
                                            Eigen3
                                            Pangolin
                                            OpenCV
                                            realsense2)
endif()

target_link_libraries(${PROJECT_NAME} ${LIBS} ${realsense2_LIBRARY} ${OpenCV_LIBS})

# Activate features in the code from the options described above.
if(PX4)
  message(STATUS "Activating PX4 integrations")
  target_compile_definitions(${PROJECT_NAME} PUBLIC PX4)
endif()
if(SMT)
  message(STATUS "Selecting parallel implementation")
  target_compile_definitions(${PROJECT_NAME} PUBLIC SMT)
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
