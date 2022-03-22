add_executable(latency latency.cpp)
target_include_directories(latency PRIVATE ${sensor_msgs_INCLUDE_DIRS})
target_link_libraries(latency PRIVATE rclcpp::rclcpp ${sensor_msgs_LIBRARIES})
# always enable optimization
target_compile_options(latency PRIVATE -O3)
