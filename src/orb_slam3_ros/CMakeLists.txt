cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV 4.2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3)
SET(CMAKE_INSTALL_RPATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib:${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

add_executable(ros_mono src/ros_mono.cpp)
ament_target_dependencies(ros_mono rclcpp cv_bridge OpenCV)
target_link_libraries(ros_mono ORB_SLAM3 DBoW2)
install(TARGETS ros_mono
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
