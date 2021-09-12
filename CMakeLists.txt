SET(CMAKE_CXX_FLAGS "-std=c++0x")

cmake_minimum_required(VERSION 2.8.3)
project(ece642rtle)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(DIRECTORY msg FILES
  PoseOrntBundle.msg
  timeInt8.msg
  bumpEcho.msg
  aendEcho.msg)
add_service_files(DIRECTORY srv FILES
                  Kill.srv
                  Spawn.srv
		  RTIbump.srv
		  RTIatend.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

set(ece642rtle_node_SRCS
  src/ece642rtle.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
  src/maze.cpp
)
set(ece642rtle_node_HDRS
  include/ece642rtle/turtle_frame.h
  include/ece642rtle/maze.h
)


# If you have additional files that need building, add them to this list
# Replace ANDREWID with your own ANDREWID!
set(ece642rtle_student_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/student/ros_turtle_interface.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/student/ANDREWID_student_maze.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/student/ANDREWID_student_turtle.cpp
  )

# Build the monitors: each target must be in quotes.
# First item is the target name, following items are the source files
# Implicitly all of these are built with monitors/ros_monitor_interface.cpp
set(monitor_TARGETS_SRCS
  "ece642rtle_logging_monitor monitors/logging_monitor.cpp"
  "ece642rtle_step_monitor monitors/step_monitor.cpp"
  )

set(ece642rtle_monitor_main monitors/ros_monitor_interface.cpp)

qt5_wrap_cpp(ece642rtle_node_MOCS ${ece642rtle_node_HDRS})


add_executable(ece642rtle_node ${ece642rtle_node_SRCS} ${ece642rtle_node_MOCS})
target_link_libraries(ece642rtle_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ece642rtle_node ece642rtle_gencpp)

add_executable(ece642rtle_student ${ece642rtle_student_SRCS})

set(proj8_flags false)

if(proj8_flags)
# Warning flags for Projects 8,9,10
target_compile_options(ece642rtle_student PUBLIC
  "-Werror" # Do not comment this out after Project 8!
  "-Wextra"
  "-Wall"
  "-Wfloat-equal"
  "-Wconversion"
  "-Wparentheses"
  "-pedantic"
  "-Wunused-parameter"
  "-Wunused-variable"
  "-Wreturn-type"
  "-Wunused-function"
  "-Wredundant-decls"
  "-Wreturn-type"
  "-Wredundant-decls"
  "-Wunused-value"
  "-Wswitch-default"
  "-Wuninitialized" "-O1" "-Winit-self"
  )
endif(proj8_flags)

target_link_libraries(ece642rtle_student ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(ece642rtle_student ece642rtle_gencpp)

foreach(monitor ${monitor_TARGETS_SRCS})
  string(REPLACE " " ";" monitor_list ${monitor})
  list(GET monitor_list 0 mon_target)
  list(REMOVE_AT monitor_list 0)
  set(mon_sources ${monitor_list})
  add_executable(${mon_target} ${ece642rtle_monitor_main} ${mon_sources})
  target_link_libraries(${mon_target} ${catkin_LIBRARIES})
  add_dependencies(${mon_target} ece642rtle_gencpp)
endforeach()

install(TARGETS ece642rtle_node ece642rtle_student
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
