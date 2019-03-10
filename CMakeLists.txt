cmake_minimum_required(VERSION 2.8)
project(diy_pid)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  TargetCoord.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(CATKIN_DEPENDS message_runtime)

#catkin_python_setup()


install(PROGRAMS src/drone_controller.py  
		 src/drone_status.py  
		 src/drone_video_display.py  
		 src/video_processing.py
		 src/joystick_controller.py  
		 src/keyboard_controller.py
		 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)