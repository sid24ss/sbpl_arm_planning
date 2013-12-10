cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(control_right_arm src/control_right_arm.cpp)
rosbuild_add_executable(control_left_arm src/control_left_arm.cpp)

rosbuild_add_executable(move_right_arm src/move_right_arm.cpp)
rosbuild_add_executable(move_left_arm src/move_left_arm.cpp)
rosbuild_add_executable(move_right_arm_multiple_goals_move_arm src/move_right_arm_multiple_goals_move_arm.cpp)
rosbuild_add_executable(move_right_arm_grasp_database src/move_right_arm_grasp_database.cpp)



rosbuild_add_executable(attachObject src/attach_object_to_gripper.cpp)
target_link_libraries(move_right_arm_grasp_database pr2_grasp_database)
