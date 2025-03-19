cmake_minimum_required(VERSION 3.20)
project(OutDoARServer)
set(CMAKE_CXX_STANDARD 20)

message(STATUS "Configuring externals...")

add_subdirectory(extern/Livox-SDK2)

message(STATUS "Externals configured.")

set(APPLICATION_NAME "OutDoARServer")

file(GLOB_RECURSE OPUTDOARSERVER_SOURCES "src/*.cpp")
file(GLOB_RECURSE OPUTDOARSERVER_HEADERS "include/*.h")

add_executable( ${APPLICATION_NAME} 
                ${OPUTDOARSERVER_SOURCES} 
                ${OPUTDOARSERVER_HEADERS})

target_include_directories(${APPLICATION_NAME} PRIVATE include)

target_link_libraries(
  ${APPLICATION_NAME}
  livox_lidar_sdk_static
)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT OutDoARServer)