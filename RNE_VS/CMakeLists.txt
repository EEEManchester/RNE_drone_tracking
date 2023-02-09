cmake_minimum_required(VERSION 3.0.2)
project(rne_vs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  apriltag_ros
  ct_core 
  ct_rbd 
  ct_optcon 
  ct_models
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_offboard_example
#  CATKIN_DEPENDS geometry_msgs mavros roscapp
  DEPENDS gazebo_ros
)


# add all catkin package
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



# define executable
add_executable(${PROJECT_NAME}_node src/rne_vs.cpp)

target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
Eigen3::Eigen
)

# define executable
add_executable(${PROJECT_NAME}_tag_navig_node src/rne_tag_navig.cpp)

target_link_libraries(${PROJECT_NAME}_tag_navig_node
${catkin_LIBRARIES}
)