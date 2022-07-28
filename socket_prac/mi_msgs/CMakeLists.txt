cmake_minimum_required(VERSION 2.8.3)
project(mi_msgs)
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

add_message_files(DIRECTORY msg
  FILES
    Car.msg
	Control.msg
	Remote.msg
	RTK.msg
	Line.msg
	Obj_state.msg
    Car_LGIT.msg
    SCH_LGIT.msg
    Trigger.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
