execute_process(COMMAND "/home/roma/roma-small-rover/build/servo_ros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/roma/roma-small-rover/build/servo_ros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
