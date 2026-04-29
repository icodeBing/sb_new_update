cmake_minimum_required(VERSION 3.12)

project(niic_ethercat_example)

# set(Boost_USE_STATIC_LIBS ON)

find_package(Boost REQUIRED program_options)

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

add_executable(cxx-sdo-request "main.cpp" "power.cpp")

target_link_libraries(
    cxx-sdo-request PRIVATE ethercat-cpp::ethercat-cpp Boost::program_options
                            spdlog::spdlog
)
