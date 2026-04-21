cmake_minimum_required(VERSION 3.12)

project(niic_ethercat_example)

if(NOT TARGET ethercat-cpp::ethercat-cpp)
    find_package(
        ethercat-cpp
        REQUIRED
        PATHS
        "${CMAKE_CURRENT_SOURCE_DIR}/../../"
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )
endif()

add_executable(ethercat-c-exipc-demo "main.c")

target_link_libraries(ethercat-c-exipc-demo PRIVATE ethercat-cpp::ethercat-cpp)

target_compile_definitions(ethercat-c-exipc-demo PRIVATE _GNU_SOURCE)
