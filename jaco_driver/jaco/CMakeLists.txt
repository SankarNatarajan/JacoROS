cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

INCLUDE(FindPkgConfig)

### Finding glib-2.0
find_package(glib REQUIRED)
IF(glib-2.0_FOUND)
        MESSAGE(STATUS "Looking for glib-2.0 - found")
ELSE(glib-2.0_FOUND)
        MESSAGE(FATAL_ERROR "Looking for glib-2.0 - not found")
ENDIF(glib-2.0_FOUND)

### Finding mono-2.0
find_package(mono REQUIRED)
IF(mono-2.0_FOUND)
        MESSAGE(STATUS "Looking for mono-2.0 - found")
ELSE(mono-2.0_FOUND)
        MESSAGE(FATAL_ERROR "Looking for mono-2.0 - not found")
ENDIF(mono-2.0_FOUND)


#action for joint movement
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CSharpWrapper_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../CSharpWrapper/CSharpWrapper/bin/Debug)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

INCLUDE_DIRECTORIES(${mono-2.0_INCLUDE_DIRS} ${glib-2.0_INCLUDE_DIRS} ${CSharpWrapper_INCLUDE_DIR})

set(Include_Libs optimized ${mono-2.0_INCLUDE_LIBS} ${glib-2.0_INCLUDE_LIBS})


rosbuild_add_boost_directories()

rosbuild_add_executable(jaco src/abstract_jaco.cpp  src/jaco_node.cpp src/jaco.cpp src/jaco_joint_publisher.cpp src/jaco_action_controller.cpp)

rosbuild_link_boost(jaco thread)
target_link_libraries(jaco ${Include_Libs})
