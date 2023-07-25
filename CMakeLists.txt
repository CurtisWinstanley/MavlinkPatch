cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED on)

project(mavgen_patch)




#set(CMAKE_TOOLCHAIN_FILE "C:/Program Files/vcpkg/scripts/buildsystems/vcpkg.cmake")

set(INCLUDES_LIST 			
	"${CMAKE_CURRENT_SOURCE_DIR}/deps"
    "${CMAKE_CURRENT_SOURCE_DIR}/generator"
)


add_executable(main  "main.cpp")
include_directories(main "${CMAKE_CURRENT_SOURCE_DIR}/messages/testMavMessage")
include_directories(main "${CMAKE_CURRENT_SOURCE_DIR}/generator")
include_directories(main "${CMAKE_CURRENT_SOURCE_DIR}/deps")