cmake_minimum_required(VERSION 3.12)
project(ethercat-cxx-addr-mode-demo)
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED program_options)
if(NOT TARGET ethercat-cpp::ethercat-cpp)
    find_package(ethercat-cpp REQUIRED)
endif()

add_executable(ethercat-cxx-addr-mode-demo "main.cpp" "power.cpp")
target_link_libraries(
    ethercat-cxx-addr-mode-demo PRIVATE ethercat-cpp::ethercat-cpp Boost::program_options
                              spdlog::spdlog
)
