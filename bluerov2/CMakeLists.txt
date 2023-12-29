cmake_minimum_required(VERSION 3.0.2)
project(bluerov2)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	Bar30.msg
	Attitude.msg
	FTSensor.msg
  State.msg
  SetDepth.msg 
  SetHeading.msg 
  SetVelocity.msg 
)
generate_dynamic_reconfigure_options(

  cfg/PositionControl.cfg
  cfg/VelocityControl.cfg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   nodes/bluerov2_node.py
   nodes/bluerov2_controller.py
   nodes/identification_motors_test.py
   nodes/test_step_motor.py
   src/bluerov2_bridge_mavlink.py
   nodes/bluerov2_teleop.py
   nodes/opencm904_node.py
   nodes/opencm904_control.py
   nodes/opencm904_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   DIRECTORY launch src msg nodes config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

