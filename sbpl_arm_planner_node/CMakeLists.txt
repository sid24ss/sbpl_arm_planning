cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

rosbuild_add_boost_directories()

#rosbuild_add_compile_flags(-g)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(sbpl_arm_planner_node src/sbpl_arm_planner_node.cpp)

rosbuild_add_executable(attachObject src/attach_object_to_right_gripper.cpp)

rosbuild_add_executable(addObjectsToMap src/add_objects_to_map.cpp)
rosbuild_add_executable(removeObjectsFromMap src/remove_objects_from_map.cpp)


target_link_libraries(sbpl_arm_planner_node sbpl_arm_planner)
target_link_libraries(sbpl_arm_planner_node visualize_arm)
target_link_libraries(sbpl_arm_planner_node ${SBPL_LIBRARIES})
rosbuild_link_boost(sbpl_arm_planner_node signals)
