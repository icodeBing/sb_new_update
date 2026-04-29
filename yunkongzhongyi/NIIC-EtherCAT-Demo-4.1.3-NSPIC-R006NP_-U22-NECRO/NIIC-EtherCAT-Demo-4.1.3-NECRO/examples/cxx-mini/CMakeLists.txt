cmake_minimum_required(VERSION 3.14)

project (ethercat-demo)

if (NOT TARGET ethercat-cpp::ethercat-cpp)
    find_package(ethercat-cpp REQUIRED)
endif ()

add_executable(ethercat-demo main.cpp)

target_link_libraries(ethercat-demo PRIVATE ethercat-cpp::ethercat-cpp)
