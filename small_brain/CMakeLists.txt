cmake_minimum_required(VERSION 3.0.2)
project(small_brain_sim)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(gazebo REQUIRED)

catkin_package()

# ── BlockFollower Gazebo World Plugin ────────────────────────────
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(block_follower_plugin SHARED src/block_follower_plugin.cpp)
target_link_libraries(block_follower_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS block_follower_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch urdf config scripts models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
