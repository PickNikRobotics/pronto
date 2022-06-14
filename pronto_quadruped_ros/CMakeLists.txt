cmake_minimum_required(VERSION 3.10.2)
project(pronto_quadruped_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pronto_core REQUIRED)
find_package(pronto_ros REQUIRED)
find_package(pronto_quadruped REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pronto_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  Boost
  pronto_core
  pronto_ros
  pronto_quadruped
  geometry_msgs
  sensor_msgs
  pronto_msgs
  visualization_msgs
  tf2_eigen
  tf2_ros
)

include_directories(include)

set(SOURCES ${CMAKE_SOURCE_DIR}/src/conversions.cpp
            ${CMAKE_SOURCE_DIR}/src/legodo_handler_ros.cpp
            ${CMAKE_SOURCE_DIR}/src/stance_estimator_ros.cpp
            ${CMAKE_SOURCE_DIR}/src/force_sensor_stance_estimator_ros.cpp
            ${CMAKE_SOURCE_DIR}/src/foot_sensor_stance_detector_ros.cpp
            ${CMAKE_SOURCE_DIR}/src/leg_odometer_ros.cpp
            ${CMAKE_SOURCE_DIR}/src/dynamic_stance_estimator_ros.cpp
            ${CMAKE_SOURCE_DIR}/src/bias_lock_handler_ros.cpp
)

# Add library
add_library(${PROJECT_NAME} SHARED ${SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
