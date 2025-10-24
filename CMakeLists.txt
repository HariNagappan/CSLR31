cmake_minimum_required(VERSION 3.28)

project(FlightOpt LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})


target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)


