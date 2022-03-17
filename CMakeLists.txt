cmake_minimum_required(VERSION 3.16)
project(canlab_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(gstreamer_app gstreamer-app-1.0 REQUIRED)

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  libclpe/include
  ${sensor_msgs_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/libclpe/lib/libclpe.a
  rclcpp::rclcpp
  ${image_transport_LIBRARIES}
  ${gstreamer_app_LIBRARIES}
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

install(TARGETS ${PROJECT_NAME})

ament_package()
