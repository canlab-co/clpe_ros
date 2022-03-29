# TODO: dummy libclpe target, libclpe should export a cmake target so that we can
# properly define dependencies.
find_package(PkgConfig REQUIRED)
pkg_search_module(gstreamer_app gstreamer-app-1.0 REQUIRED)

add_library(clpe INTERFACE)
target_include_directories(clpe INTERFACE libclpe/include)
target_link_libraries(clpe INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/libclpe/lib/libclpe.a)
