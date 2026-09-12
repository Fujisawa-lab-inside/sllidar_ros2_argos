################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(sllidar_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

set(SLLIDAR_SDK_PATH "./sdk/")

FILE(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

################################################################################
# Build
################################################################################
add_executable(sllidar_node	src/sllidar_node.cpp ${SLLIDAR_SDK_SRC})
target_include_directories(sllidar_node SYSTEM PRIVATE
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)

# target_link_libraries(sllidar_node ${ament_cmake_LIBRARIES})
ament_target_dependencies(sllidar_node
  rclcpp
  std_srvs
  sensor_msgs
)

add_executable(sllidar_client src/sllidar_client.cpp)
ament_target_dependencies(sllidar_client
  rclcpp
  std_srvs
  sensor_msgs
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(
    src/sllidar_node.cpp
    src/sllidar_client.cpp
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic"
  )
endif()
################################################################################
# Install
################################################################################
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS sllidar_node sllidar_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  enable_testing()
  add_executable(sllidar_diagnostics_test test/test_scan_diagnostics.cpp)
  target_include_directories(sllidar_diagnostics_test PRIVATE src ${SLLIDAR_SDK_PATH}/include)
  target_link_libraries(sllidar_diagnostics_test Threads::Threads)
  add_test(NAME sllidar_scan_diagnostics COMMAND sllidar_diagnostics_test)
  add_executable(sllidar_decode_evidence_test test/test_decode_error_evidence.cpp ${SLLIDAR_SDK_SRC})
  target_include_directories(sllidar_decode_evidence_test PRIVATE
    ${SLLIDAR_SDK_PATH}/include ${SLLIDAR_SDK_PATH}/src)
  target_link_libraries(sllidar_decode_evidence_test Threads::Threads)
  add_test(NAME sllidar_decode_error_evidence COMMAND sllidar_decode_evidence_test)
endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()
