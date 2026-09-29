cmake_minimum_required(VERSION 3.10)
include_guard(GLOBAL)

project(skywatcher LANGUAGES C)

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_library(skywatcher STATIC)
add_subdirectory(../logging ${CMAKE_BINARY_DIR}/libs_build/logging_from_skywatcher)
add_subdirectory(../threading ${CMAKE_BINARY_DIR}/libs_build/threading_from_skywatcher)
add_subdirectory(../string ${CMAKE_BINARY_DIR}/libs_build/string_from_skywatcher)
add_subdirectory(../api ${CMAKE_BINARY_DIR}/libs_build/api_from_skywatcher)
add_subdirectory(../socket ${CMAKE_BINARY_DIR}/libs_build/socket_from_skywatcher)
add_subdirectory(../physics ${CMAKE_BINARY_DIR}/libs_build/physics_from_skywatcher)

target_sources(skywatcher PRIVATE
	skywatcher.c
	skywatcher_communication.c
)

target_include_directories(skywatcher PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(skywatcher
	PUBLIC
		logging
		threading
		string
		api
		socket
		physics
)
