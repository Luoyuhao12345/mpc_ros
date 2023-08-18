cmake_minimum_required(VERSION 3.0.2)
project(my_mpc)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  tf
)

# add_message_files(
#           FILES
#           way_point.msg
#           car_info.msg
#           my_marker.msg
#           way_points.msg
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

catkin_package(
  CATKIN_DEPENDS  roscpp rospy std_msgs tf message_runtime 
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 include
)

link_directories(
  /usr/local/lib 
  )



add_executable(car_controller
  src/car_obj.cpp
  src/util.cpp
  src/mpc_util.cpp
  src/car_controller.cpp
)
add_dependencies(car_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_controller
  ${catkin_LIBRARIES}
  libosqp.so  
  libOsqpEigen.so 
  libqdldl.so 
)