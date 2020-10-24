cmake_minimum_required(VERSION 2.8.3)
project(jetson_nano_mouse_line_following)

find_package(catkin REQUIRED COMPONENTS
  roslint
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslint_python()
roslint_add_test()
