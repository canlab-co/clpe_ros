add_executable(latency latency.cpp)
target_include_directories(latency PRIVATE ${roscpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})
target_link_libraries(latency PRIVATE ${roscpp_LIBRARIES} ${sensor_msgs_LIBRARIES})
# always build in release mode
separate_arguments(BUILD_FLAGS NATIVE_COMMAND ${CMAKE_CXX_FLAGS_RELEASE})
target_compile_options(latency PRIVATE ${BUILD_FLAGS})
