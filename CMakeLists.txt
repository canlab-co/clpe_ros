cmake_minimum_required(VERSION 3.16)
project(clpe_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(clpe REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(clpe_ros_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib INTERFACE)
target_include_directories(${PROJECT_NAME}_lib INTERFACE
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${clpe_ros_msgs_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_lib INTERFACE
  clpe::clpe
  rclcpp::rclcpp
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${clpe_ros_msgs_LIBRARIES}
)
target_compile_features(${PROJECT_NAME}_lib INTERFACE cxx_std_14)
add_executable(${PROJECT_NAME}
  src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_lib)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})

###############################################################################
# ROS2 component
###############################################################################

include(GenerateExportHeader)
find_package(rclcpp_components REQUIRED)

# Create shared library
add_library(${PROJECT_NAME}_component SHARED
  src/component.cpp)
target_compile_definitions(${PROJECT_NAME}_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${rclcpp_components_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_lib
  ${rclcpp_components_LIBRARIES})
set_target_properties(${PROJECT_NAME}_component PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)

# Generate export headers
GENERATE_EXPORT_HEADER(${PROJECT_NAME}_component)

# register component
rclcpp_components_register_nodes(${PROJECT_NAME}_component "clpe::ClpeComponentNode")
set(node_plugins "${node_plugins}clpe::ClpeComponentNode;$<TARGET_FILE:${PROJECT_NAME}_component>\n")
file(GENERATE
  OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
  CONTENT "${node_plugins}")

# install target
install(TARGETS ${PROJECT_NAME}_component)

###############################################################################

option(CLPE_ROS_BUILD_BENCHMARKS "build benchmarks")
if (${CLPE_ROS_BUILD_BENCHMARKS})
  add_subdirectory(benchmarks)
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  FILES fastdds.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
