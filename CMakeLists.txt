cmake_minimum_required(VERSION 2.8.3)
project(turtle_325)


find_package(catkin REQUIRED COMPONENTS
   geometry_msgs
  roscpp
  turtlesim
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/turtle_draws.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
