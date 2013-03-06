cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

link_libraries(liblapack.so)    # lapack is need by openrave ikfast

rosbuild_add_library(jaco_openrave_ikfast src/jaco_openrave_ikfast.cpp)

rosbuild_add_library(${PROJECT_NAME} src/jaco_arm_kinematics.cpp src/jaco_arm_kinematics_plugin.cpp )
target_link_libraries(${PROJECT_NAME} jaco_openrave_ikfast)

rosbuild_add_executable(${PROJECT_NAME}_node src/main.cpp )
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
