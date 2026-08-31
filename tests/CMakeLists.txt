cmake_minimum_required(VERSION 3.10)

project(skywatcher_tests LANGUAGES C)

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(skywatcher_tests)

add_subdirectory(../ ${CMAKE_BINARY_DIR}/skywatcher)

target_sources(skywatcher_tests PRIVATE
	main.c
)

target_link_libraries(skywatcher_tests
	PRIVATE
		skywatcher
		m
)
