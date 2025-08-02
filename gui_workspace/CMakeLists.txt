cmake_minimum_required(VERSION 3.5)

project(gui_workspace VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(OpenCV REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Network)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(PROJECT_SOURCES
        robotarmclient.h
        robotarmclient.cpp
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        teachpanel.h teachpanel.cpp teachpanel.ui
        protocolparser.h protocolparser.cpp
        messagehandler.h messagehandler.cpp
        protocolconstants.h
        messagetypes.h
        clienttypes.h
)

qt_add_executable(gui_workspace
        ${PROJECT_SOURCES}
    )

target_link_libraries(gui_workspace PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(gui_workspace PROPERTIES
    WIN32_EXECUTABLE TRUE
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

set(QT_VERSION Qt6 CACHE STRING "The Qt version framework to use (Qt5 or Qt6).")
set(BREEZE_EXTENSIONS all CACHE STRING "The extensions to include in our stylesheets.")
set(BREEZE_STYLES all CACHE STRING "The styles to include in our stylesheets.")

include(breeze)

target_include_directories(gui_workspace PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gui_workspace PRIVATE Qt${QT_VERSION_MAJOR}::Widgets breeze Qt${QT_VERSION_MAJOR}::Network ${OpenCV_LIBS})

install(TARGETS gui_workspace
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
