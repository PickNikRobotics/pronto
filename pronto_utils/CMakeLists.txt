cmake_minimum_required(VERSION 3.10.2)
project(pronto_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# TODO: Enable compiler warnings

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  filesystem
)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(FILTER_TOOLS_LIB ${PROJECT_NAME})
set(BACKLASH_FILTER_LIB backlash_filter)
set(KALMAN_FILTER_LIB kalman_filter)

add_library(${PROJECT_NAME} SHARED src/Filter.cpp
                            src/HeavyLowPassFilter.cpp
                            #src/SignalTap.cpp
                            src/DataFileLogger.cpp
                            src/BipolarSchmittTrigger.cpp
                            src/MedianFilter.cpp
                            src/RateChange.cpp
                            src/TrapezoidalInt.cpp
                            src/DistributedDiff.cpp
                            src/NumericalDiff.cpp
                            src/ExpireTimer.cpp
                            src/SchmittTrigger.cpp
                            src/BipolarSchmittTrigger.cpp
                            src/Gaussian.cpp)

add_library(${KALMAN_FILTER_LIB} SHARED src/alpha_filter.cpp
                                 src/kalman_filter.cpp
                                 src/simple_kalman_filter.cpp)

add_library(${BACKLASH_FILTER_LIB} SHARED src/backlash_filter.cpp
                                   src/torque_adjustment.cpp)

target_link_libraries(${BACKLASH_FILTER_LIB} ${KALMAN_FILTER_LIB})

######################################################
#add_executable(test-kalman-filter src/kalman_filter_demo.cpp)
#target_link_libraries(test-kalman-filter ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(test-filter src/filtering_demo.cpp)
#target_link_libraries(test-filter ${catkin_LIBRARIES} backlash_filter_tools)

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
endif()

install(
  TARGETS ${PROJECT_NAME} ${KALMAN_FILTER_LIB} ${BACKLASH_FILTER_LIB}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()
