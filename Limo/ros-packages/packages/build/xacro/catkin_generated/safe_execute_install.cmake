execute_process(COMMAND "/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/xacro/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/xacro/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
