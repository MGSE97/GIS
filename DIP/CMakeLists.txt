cmake_minimum_required(VERSION 3.0.0)
project(gis VERSION 0.1.0)

#include(CTest)
#enable_testing()

find_package( OpenCV REQUIRED )

add_executable(gis main.cpp)

target_link_libraries(gis ${OpenCV_LIBS} )

#set(CPACK_PROJECT_NAME ${PROJECT_NAME})
#set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
#include(CPack)
