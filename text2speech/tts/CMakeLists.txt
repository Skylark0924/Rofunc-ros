cmake_minimum_required(VERSION 2.8.3)
project(tts)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation rospy rosunit std_msgs sound_play)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(FILES Synthesizer.srv Polly.srv)

## Generate actions in the 'action' folder
add_action_files(FILES Speech.action)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES tts
  CATKIN_DEPENDS actionlib_msgs message_runtime rospy std_msgs
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/polly_node.py
  scripts/synthesizer_node.py
  scripts/tts_node.py
  scripts/voicer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  ## Add folders to be run by python nosetests
  catkin_add_nosetests(test/test_unit_synthesizer.py)
  catkin_add_nosetests(test/test_unit_polly.py)
  
  if(BUILD_AWS_TESTING)
      find_package(rostest REQUIRED COMPONENTS tts)
      add_rostest(test/integration_tests.test DEPENDENCIES ${tts_EXPORTED_TARGETS})
  endif()
endif()


