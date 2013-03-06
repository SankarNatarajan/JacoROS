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

# getting fk solver info
rosbuild_add_executable(get_solver_info test/get_solver_info.cpp)
# getting fk
rosbuild_add_executable(get_fk test/get_fk.cpp)
# getting ik
rosbuild_add_executable(get_ik test/get_ik.cpp)
# moving the jaco joint using move arm
rosbuild_add_executable(move_jaco_joint_goal test/move_jaco_joint_goal.cpp)
# moving the jaco in cartesian space using move arm
rosbuild_add_executable(move_arm_simple_pose_goal test/move_arm_simple_pose_goal.cpp)
# moving the jaco in cartesian space using move arm
rosbuild_add_executable(move_arm_relative_pose_goal test/move_jaco_simple_pose_goal_fk.cpp)



