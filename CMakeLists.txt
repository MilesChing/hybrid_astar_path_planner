cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)

## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection.cpp)

find_package(OpenCV 3.3.0 EXACT)
find_package(OMPL REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)
include_directories(include ${OMPL_INCLUDE_DIRS})
add_executable(test ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp ${SOURCES})
target_link_libraries(test ${OMPL_LIBRARIES} ${OpenCV_LIBS})
