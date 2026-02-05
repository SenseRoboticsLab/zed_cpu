cmake_minimum_required(VERSION 3.5)
project(zed_cpu_ros2)

# Default to C++14 or newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Embedded ARM definitions (preserved from original)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_options(-mtune=cortex-a72 -mcpu=cortex-a72)
    add_definitions(-DEMBEDDED_ARM)
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
    add_compile_options(-mfloat-abi=hard -mfpu=neon-fp-armv8 -mneon-for-64bits -mtune=cortex-a72)
    add_definitions(-DEMBEDDED_ARM)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(HIDAPI REQUIRED)
find_package(LIBUSB 1 REQUIRED)
# OpenCV is usually found via cv_bridge transitively, but good to be explicit if using libs
find_package(OpenCV REQUIRED)

include_directories(
  include
  include/zed_cpu_ros2
  ${HIDAPI_INCLUDE_DIRS}
  ${LibUSB_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# --- ZED Library (Hardware Interface) ---
add_library(zed_lib_ros2
  src/zed_lib/videocapture.cpp
  src/zed_lib/sensorcapture.cpp
)
target_include_directories(zed_lib_ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/zed_cpu_ros2>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/zed_cpu_ros2>
)
target_link_libraries(zed_lib_ros2
  ${OpenCV_LIBS}
  ${LibUSB_LIBRARIES}
  ${HIDAPI_LIBRARIES}
)

# --- ZED Node ---
add_executable(zed_node_ros2
  src/main.cpp
  src/zed_cpu.cpp
)
target_link_libraries(zed_node_ros2
  zed_lib_ros2
  ${OpenCV_LIBS}
)
ament_target_dependencies(zed_node_ros2
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

# --- Install ---
install(TARGETS zed_lib_ros2 zed_node_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/zed2i_rectification_node_new.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME zed2i_rectification_node
)

ament_package()
