# CMake generated Testfile for 
# Source directory: /home/alejandro/bookros2_ws/src/book_ros2/br2_tracking/tests
# Build directory: /home/alejandro/bookros2_ws/build/br2_tracking/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(pid_test "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/alejandro/bookros2_ws/build/br2_tracking/test_results/br2_tracking/pid_test.gtest.xml" "--package-name" "br2_tracking" "--output-file" "/home/alejandro/bookros2_ws/build/br2_tracking/ament_cmake_gtest/pid_test.txt" "--command" "/home/alejandro/bookros2_ws/build/br2_tracking/tests/pid_test" "--gtest_output=xml:/home/alejandro/bookros2_ws/build/br2_tracking/test_results/br2_tracking/pid_test.gtest.xml")
set_tests_properties(pid_test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/alejandro/bookros2_ws/build/br2_tracking/tests/pid_test" TIMEOUT "60" WORKING_DIRECTORY "/home/alejandro/bookros2_ws/build/br2_tracking/tests" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/alejandro/bookros2_ws/src/book_ros2/br2_tracking/tests/CMakeLists.txt;2;ament_add_gtest;/home/alejandro/bookros2_ws/src/book_ros2/br2_tracking/tests/CMakeLists.txt;0;")
subdirs("../gtest")
