cmake_minimum_required(VERSION 2.8.11)

# ALLOWS MORE READABLE "IF THEN ELSE" CONSTRUCTS ------------
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

project (OmnibotRecognise)

set(CMAKE_BUILD_TYPE Release)

set(NCS_ROBOT_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../NCSRobotLib")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -DDEVICE_MN256R1")

# add Qt and OpenGL

# -- In case CMake cannot find the Qt5 install prefix ask the user to set it manually
# (put back in when you want to visualize; needs installation of Qt on parallella)
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/qt5")

message(STATUS "=== Searching for packages Boost") # Qt5Network, 

# find_package(Qt5Network REQUIRED) # (put back in when you want to visualize, needs installation of Qt on parallella)

#add Boost
find_package(Boost REQUIRED COMPONENTS program_options system thread date_time filesystem)
message(STATUS Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR} ../include)

# add RobotLib
include_directories(
	${NCS_ROBOT_LIB_DIR}/include
	${NCS_ROBOT_LIB_DIR}/aerctl
	${NCS_ROBOT_LIB_DIR}/cjson)

link_directories(${NCS_ROBOT_LIB_DIR}/build)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/build)

# add project files (compile a library with all the external classes)
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include)
#	${CMAKE_CURRENT_SOURCE_DIR}/QtNetwork)
file(GLOB SOURCES	
	"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"  
	"${NCS_ROBOT_LIB}/cjson/*.cpp")
#	"${CMAKE_CURRENT_SOURCE_DIR}/QtNetwork/*.c")
add_library(${PROJECT_NAME}_lib SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME}_lib ${Boost_LIBRARIES} Robot-lib pthread) # Qt5::Network (put back in when you want to visualize, needs installation of Qt on parallella)

# build executable
file(GLOB SOURCES
	"${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")
add_executable(OmnibotRecognise ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib Robot-lib ${Boost_LIBRARIES} pthread m) # aerctl Qt5::Network (put back in when you want to visualize, needs installation of Qt on parallella)
