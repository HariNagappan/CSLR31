cmake_minimum_required(VERSION 3.28)

project(FlightOpt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Collect all source files for the library
file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/Algo_Templates/*.cpp")

# Create shared library
add_library(FlightOptLib SHARED ${SRC_FILES})

# Include your own headers
target_include_directories(FlightOptLib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Use pkg-config to find dependencies
find_package(PkgConfig REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(CURL REQUIRED libcurl)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

# Include directories from pkg-config
target_include_directories(FlightOptLib PRIVATE
    ${JSONCPP_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Link libraries (Eigen3 is header-only)
target_link_libraries(FlightOptLib
    PRIVATE
        ${JSONCPP_LIBRARIES}
        ${CURL_LIBRARIES}
)

# Optional: apply compiler flags from pkg-config
target_compile_options(FlightOptLib PRIVATE
    ${JSONCPP_CFLAGS_OTHER}
    ${CURL_CFLAGS_OTHER}
    ${EIGEN3_CFLAGS_OTHER}
)

# Now create the executable
add_executable(FlightOptExec main.cpp)

# Include library headers for the executable
target_include_directories(FlightOptExec PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link the executable to the shared library
target_link_libraries(FlightOptExec PRIVATE FlightOptLib)
