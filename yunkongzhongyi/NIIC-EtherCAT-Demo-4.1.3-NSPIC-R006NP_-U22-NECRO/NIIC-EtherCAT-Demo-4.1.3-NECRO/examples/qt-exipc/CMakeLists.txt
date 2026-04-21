cmake_minimum_required(VERSION 3.14)

project(QT-Test)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_Release ${CMAKE_BINARY_DIR}/Release/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_Debug ${CMAKE_BINARY_DIR}/Debug/bin)

find_package(Qt5 REQUIRED Widgets)


add_executable(qt_test
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    receivethread.cpp
    receivethread.h
    servoform.cpp
    servoform.h
    servoform.ui
    sendthread.cpp
    sendthread.h
    )
target_link_libraries(qt_test
    PRIVATE
    Qt5::Widgets
    )
set_target_properties(qt_test
    PROPERTIES  AUTOMOC ON
                AUTOUIC ON
                AUTORCC ON
    )
