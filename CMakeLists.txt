cmake_minimum_required(VERSION 3.28)

project(FlightOpt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Executable and library names
set(EXECUTABLE FlightOptExec)
set(SHARED_LIBRARY_FlightOpt FlightOptLib)

# Find packages via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(CURL REQUIRED libcurl)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

# Header-only library
add_library(${SHARED_LIBRARY_FlightOpt} INTERFACE)

# Include directories for header-only library
target_include_directories(${SHARED_LIBRARY_FlightOpt} INTERFACE
    ${JSONCPP_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/Algo_Templates
)

# Link libraries for header-only library
target_link_libraries(${SHARED_LIBRARY_FlightOpt} INTERFACE
    ${JSONCPP_LIBRARIES}
    ${CURL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)

# Executable target
add_executable(${EXECUTABLE} Algo_Templates/main.cpp)

# Link the header-only library to the executable
target_link_libraries(${EXECUTABLE} PRIVATE ${SHARED_LIBRARY_FlightOpt})

# Optional: Output directory for libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
