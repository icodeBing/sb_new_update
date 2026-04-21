cmake_minimum_required(VERSION 3.12)

project(niic_ethercat_example)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(
    ethercat-cpp
    PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/../"
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

foreach(
    directory
    api;c;c-exipc;c-io;cxx;cxx-2master;cxx-5master;cxx-addr-mode;cxx-mini;cxx-multi-master;cxx-restart;cxx-sdo-request;general;general-array;xmlParser;qt-exipc;cxx-exipc;cxx-auto-register;cxx-5master-ZCAI;ros_ethercat;general-xhand;general-encos;cxx-2master-encos;Ti5-Encos-Single;Ti5-Encos-2master;Ti5-Encos-3master;
)
    add_subdirectory(${directory})
endforeach()
