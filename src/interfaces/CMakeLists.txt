cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Uuid.msg"
  "msg/KeyFrameBowVector.msg"
  "msg/NewKeyFrames.msg"
  "msg/NewKeyFrameBows.msg"
  "msg/SuccessfullyMerged.msg"
  "msg/MapPoint.msg"
  "msg/IsLostFromBaseMap.msg"
  "msg/LoopClosureTriggers.msg"
  "msg/Sim3Transform.msg"
  "msg/Sim3TransformStamped.msg"
  "srv/GetCurrentMap.srv"
  "srv/AddMap.srv"
  "srv/GetMapPoints.srv"
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
)

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


ament_export_dependencies(rosidl_default_runtime)
ament_package()
