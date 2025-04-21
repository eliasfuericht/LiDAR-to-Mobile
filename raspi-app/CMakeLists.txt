cmake_minimum_required(VERSION 3.20)
project(OutDoARServer)
set(CMAKE_CXX_STANDARD 20)

message(STATUS "Configuring externals...")

#add_subdirectory(extern/Livox-SDK2)

# add_subdirectory(extern/Open3D)
# linking prebuild Open3D since it takes forever to build on raspi
add_library(Open3D::Open3D STATIC IMPORTED)
set_target_properties(Open3D::Open3D PROPERTIES
    IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/extern/prebuilt/Open3D/lib/libOpen3D.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/extern/prebuilt/Open3D/include"
)

message(STATUS "Externals configured.")

set(APPLICATION_NAME "OutDoARServer")

file(GLOB_RECURSE OPUTDOARSERVER_SOURCES "src/test.cpp")
file(GLOB_RECURSE OPUTDOARSERVER_HEADERS "include/*.h")

add_executable( ${APPLICATION_NAME} 
                ${OPUTDOARSERVER_SOURCES} 
                ${OPUTDOARSERVER_HEADERS})

target_include_directories(${APPLICATION_NAME} PRIVATE include)

target_link_libraries(
  ${APPLICATION_NAME}
  Open3D::Open3D
)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT OutDoARServer)

# copy config.json to build directory after build finished
add_custom_command(TARGET ${APPLICATION_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy 
    ${CMAKE_SOURCE_DIR}/extern/Livox-SDK2/samples/livox_lidar_quick_start/config.json
    ${CMAKE_BINARY_DIR}/
)