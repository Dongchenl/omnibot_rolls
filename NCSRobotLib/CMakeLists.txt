cmake_minimum_required(VERSION 2.8)

project (Robot-lib)

set(CMAKE_MACOSX_RPATH 1)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -pthread -DDEVICE_MN256R1")

#add OpenCV
find_package( OpenCV REQUIRED )

#add boost
find_package(Boost REQUIRED COMPONENTS
			system
			thread
			date_time
			chrono)

include_directories(
    ${Boost_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/aerctl
)

# compile a library with all sources and headers
file(GLOB SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/aerctl/aerctl.cpp
)
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OpenCV_LIBS})
