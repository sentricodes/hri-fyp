cmake_minimum_required(VERSION 3.8)
project(vzense_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) # v ROS dependencies v
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Vzense SDK paths
set(VZENSE_SDK_ROOT "$ENV{HOME}/Documents/vsense/Vzense_SDK_Linux-3.5.4/Ubuntu18.04")
set(VZENSE_SDK_INCLUDE_DIR "${VZENSE_SDK_ROOT}/Include")
set(VZENSE_SDK_LIB_DIR "${VZENSE_SDK_ROOT}/Lib")
set(VZENSE_SDK_LIBRARY "${VZENSE_SDK_LIB_DIR}/libvzense_api.so")

add_executable(DCAM710_node src/DCAM710_node.cpp)

target_include_directories(DCAM710_node PRIVATE
  ${VZENSE_SDK_INCLUDE_DIR}
)

ament_target_dependencies(DCAM710_node
  rclcpp
  sensor_msgs
)

target_link_libraries(DCAM710_node
  ${VZENSE_SDK_LIB_DIR}/libvzense_api.so
)

set_target_properties(DCAM710_node PROPERTIES
  BUILD_RPATH "${VZENSE_SDK_LIB_DIR}"
  INSTALL_RPATH "${VZENSE_SDK_LIB_DIR}"
)

install(TARGETS
  DCAM710_node
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
