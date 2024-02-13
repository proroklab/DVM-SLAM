cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O0")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O0")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-w)

find_package(OpenCV 4.2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(interfaces REQUIRED)  

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

add_library(orb_slam3_wrapper src/orb_slam3_wrapper.cpp src/peer.cpp)
ament_target_dependencies(orb_slam3_wrapper rclcpp cv_bridge OpenCV geometry_msgs visualization_msgs nav_msgs tf2_ros image_transport interfaces)
target_link_libraries(orb_slam3_wrapper ORB_SLAM3 DBoW2)

add_executable(ros_mono src/ros_mono.cpp)
ament_target_dependencies(ros_mono rclcpp cv_bridge OpenCV rclcpp cv_bridge OpenCV geometry_msgs visualization_msgs nav_msgs tf2_ros image_transport)
target_link_libraries(ros_mono ORB_SLAM3 DBoW2 orb_slam3_wrapper)
install(TARGETS ros_mono
  DESTINATION lib/${PROJECT_NAME})

add_executable(ros_imu_mono src/ros_imu_mono.cpp)
ament_target_dependencies(ros_imu_mono rclcpp cv_bridge OpenCV rclcpp cv_bridge OpenCV geometry_msgs visualization_msgs nav_msgs tf2_ros image_transport)
target_link_libraries(ros_imu_mono ORB_SLAM3 DBoW2 orb_slam3_wrapper)
install(TARGETS ros_imu_mono
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
