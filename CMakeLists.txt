cmake_minimum_required(VERSION 3.16)
project(canlab_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(gstreamer_app gstreamer-app-1.0 REQUIRED)

add_library(${PROJECT_NAME}_lib INTERFACE)
target_include_directories(${PROJECT_NAME}_lib INTERFACE
  libclpe/include
  ${sensor_msgs_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/libclpe/lib/libclpe.a
  rclcpp::rclcpp
  ${image_transport_LIBRARIES}
  ${gstreamer_app_LIBRARIES})
target_compile_features(${PROJECT_NAME}_lib INTERFACE cxx_std_14)

add_executable(${PROJECT_NAME}
  src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_lib)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})

# TODO: This doesn't work because the clpe library is not position independent.
# include(GenerateExportHeader)
# find_package(rclcpp_components REQUIRED)
# add_library(${PROJECT_NAME}_component SHARED
#   src/component.cpp)
# target_compile_definitions(${PROJECT_NAME}_component
#   PRIVATE "COMPOSITION_BUILDING_DLL")
# target_include_directories(${PROJECT_NAME}_component PRIVATE
#   ${CMAKE_CURRENT_BINARY_DIR}
#   ${rclcpp_components_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME}_component
#   ${PROJECT_NAME}_lib
#   ${rclcpp_components_LIBRARIES})
# set_target_properties(${PROJECT_NAME}_component PROPERTIES
#   CXX_VISIBILITY_PRESET hidden
#   VISIBILITY_INLINES_HIDDEN ON)
# GENERATE_EXPORT_HEADER(${PROJECT_NAME}_component)
# rclcpp_components_register_nodes(${PROJECT_NAME}_component "clpe::ClpeNode")
# set(node_plugins "${node_plugins}canlab::ClpeDriver;$<TARGET_FILE:${PROJECT_NAME}_component>\n")
# file(GENERATE
#   OUTPUT
#     "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
#   CONTENT "${node_plugins}")

option(CANLAB_ROS_BUILD_BENCHMARKS "build benchmarks")
if (${CANLAB_ROS_BUILD_BENCHMARKS})
  add_subdirectory(benchmarks)
endif()

ament_package()
