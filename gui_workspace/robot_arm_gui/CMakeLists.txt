cmake_minimum_required(VERSION 3.5)

project(robot_arm_gui VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Network)

set(PROJECT_SOURCES
        robotarmclient.h
        robotarmclient.cpp
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 5)
    qt_add_executable(robot_arm_gui
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET robot_arm_gui APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(robot_arm_gui SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(robot_arm_gui
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(robot_arm_gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(robot_arm_gui PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

set(QT_VERSION Qt6 CACHE STRING "The Qt version framework to use (Qt5 or Qt6).")
set(BREEZE_EXTENSIONS all CACHE STRING "The extensions to include in our stylesheets.")
set(BREEZE_STYLES all CACHE STRING "The styles to include in our stylesheets.")

include(breeze)

# add_executable(myapp WIN32 MACOSX_BUNDLE "main.cpp")
target_link_libraries(robot_arm_gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets breeze Qt${QT_VERSION_MAJOR}::Network)

install(TARGETS robot_arm_gui
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(QT_VERSION_MAJOR EQUAL 5)
    qt_finalize_executable(robot_arm_gui)
endif()
