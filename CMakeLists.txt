cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Polygons__Based_Registration)

set(CMAKE_BUILD_TYPE Release)    
find_package(CGAL REQUIRED Qt5 Core)
    message(STATUS "Found CGAL: CGAL_INCLUDE_DIRS=" ${CGAL_INCLUDE_DIRS} " and CGAL_LIBRARY=" ${CGAL_LIBRARY} " and CGAL_USE_FILE=" ${CGAL_USE_FILE})
    include(${CGAL_USE_FILE})

 add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)



find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui OpenGL Xml)
set(Qt5_TARGET_LIB Qt5::Widgets Qt5::Gui Qt5::OpenGL Qt5::Xml)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(include/)

file(GLOB HEADERS
    include/*.hpp
    
    

)
file(GLOB SOURCES src/internal/*.cpp )

add_executable(PolyDetect ./src/PolyDetect.cpp ${HEADERS} ${SOURCES})
target_link_libraries ( PolyDetect  ${PCL_LIBRARIES}${CGAL_LIBRARY} ${Qt5_TARGET_LIB})

