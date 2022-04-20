cmake_minimum_required(VERSION 3.16)
project(clpe_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED)
find_package(clpe REQUIRED)
find_package(roscpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(clpe_ros_msgs REQUIRED)

catkin_package(DEPENDS roscpp image_transport clpe_ros_msgs)

add_library(${PROJECT_NAME}_lib INTERFACE)
target_include_directories(${PROJECT_NAME}_lib INTERFACE
  ${roscpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${clpe_ros_msgs_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_lib INTERFACE
  clpe
  ${roscpp_LIBRARIES}
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

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

option(CLPE_ROS_BUILD_BENCHMARKS "build benchmarks")
if (${CLPE_ROS_BUILD_BENCHMARKS})
  add_subdirectory(benchmarks)
endif()
