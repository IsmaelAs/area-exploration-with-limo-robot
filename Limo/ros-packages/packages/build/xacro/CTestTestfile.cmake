# CMake generated Testfile for 
# Source directory: /home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/src/xacro
# Build directory: /home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/xacro
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_xacro_nosetests_test "/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/test_results/xacro/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/test_results/xacro" "/usr/bin/nosetests3 -P --process-timeout=60 --where=/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/src/xacro/test --with-xunit --xunit-file=/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/test_results/xacro/nosetests-test.xml")
set_tests_properties(_ctest_xacro_nosetests_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/src/xacro/CMakeLists.txt;20;catkin_add_nosetests;/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/src/xacro/CMakeLists.txt;0;")
subdirs("test")
