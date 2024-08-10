cmake_minimum_required(VERSION 3.10)

project(skywatcher LANGUAGES C)

pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.4 gstreamer-sdp-1.0>=1.4 gstreamer-app-1.0>=1.4 gstreamer-video-1.0>=1.4)

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_library(skywatcher STATIC)

target_sources(skywatcher PRIVATE
    skywatcher.c
    skywatcher_communication.c
)

target_include_directories(skywatcher PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../api
    ${CMAKE_CURRENT_LIST_DIR}/../socket
    ${CMAKE_CURRENT_LIST_DIR}/../string
    ${CMAKE_CURRENT_LIST_DIR}/../threading
    ${CMAKE_CURRENT_LIST_DIR}/../logging
    ${CMAKE_CURRENT_LIST_DIR}/../physics
    ${CMAKE_CURRENT_LIST_DIR}/../vector
    ${GST_INCLUDE_DIRS}
)

target_link_libraries(skywatcher
    physics
    socket
)
