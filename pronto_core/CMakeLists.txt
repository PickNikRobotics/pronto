cmake_minimum_required(VERSION 3.10.2)
project(pronto_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# TODO: Enable compiler warnings

set(PRONTO_CORE_LIB ${PROJECT_NAME})

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(${PRONTO_CORE_LIB} SHARED src/rbis.cpp
                               src/rotations.cpp
                               src/rigidbody.cpp
                               src/ins_module.cpp
                               src/scan_matcher_module.cpp
                               src/rbis_update_interface.cpp
                               src/state_est.cpp
                               src/update_history.cpp
                               src/visual_odometry_module.cpp
                               src/lidar_odometry_module.cpp
                               src/vicon_module.cpp
                               src/indexed_meas_module.cpp
                               src/init_message_module.cpp
                               src/gps_module.cpp
                               src/pose_meas_module.cpp)

target_include_directories(${PRONTO_CORE_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PRONTO_CORE_LIB} Eigen3::Eigen)

ament_export_include_directories(include)

ament_export_targets(export_${PRONTO_CORE_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # TODO (vatanaksoytezer): Enable tests when I have more time
  # catkin_add_gtest(ins_module_ballistic test/imu_module_test.cpp)
  # target_link_libraries(ins_module_ballistic ${catkin_LIBRARIES} ${PRONTO_CORE_LIB} pthread)
endif()

install(
  TARGETS ${PRONTO_CORE_LIB}
  EXPORT export_${PRONTO_CORE_LIB}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
        DESTINATION include
)

ament_package()
