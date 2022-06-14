cmake_minimum_required(VERSION 3.5)
project(pronto_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Disable Wredundant-decls warnings since rosidl generates redundant function
#  declarations
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-redundant-decls")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/VisualOdometryUpdate.msg"
  "msg/FilterState.msg"
  "msg/GPSData.msg"
  "msg/IndexedMeasurement.msg"
  "msg/QuadrupedStance.msg"
  "msg/JointStateWithAcceleration.msg"
  "msg/LidarOdometryUpdate.msg"
  "msg/ControllerFootContact.msg"
  "msg/BipedForceTorqueSensors.msg"
  "msg/QuadrupedForceTorqueSensors.msg"
  "msg/VelocityWithSigmaBounds.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
