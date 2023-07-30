cmake_minimum_required(VERSION 3.5)
project(Fiducial_navig)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )



set(dependencies "geometry_msgs" "rclcpp")
add_executable(Aruco_Vision src/Aruco_Vision.cpp)
ament_target_dependencies(Aruco_Vision rclcpp std_msgs OpenCV ${dependencies})



add_executable(Aruco_Server src/Aruco_Server.cpp)
ament_target_dependencies(Aruco_Server ${dependencies})



install(TARGETS
  Aruco_Vision
  Aruco_Server
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(OpenCV REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()