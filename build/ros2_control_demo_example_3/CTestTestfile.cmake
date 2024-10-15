# CMake generated Testfile for 
# Source directory: /home/hans/ros2_ws/src/ros2_control_demos/example_3
# Build directory: /home/hans/ros2_ws/build/ros2_control_demo_example_3
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(example_3_urdf_xacro "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/hans/ros2_ws/build/ros2_control_demo_example_3/test_results/ros2_control_demo_example_3/example_3_urdf_xacro.xunit.xml" "--package-name" "ros2_control_demo_example_3" "--output-file" "/home/hans/ros2_ws/build/ros2_control_demo_example_3/ament_cmake_pytest/example_3_urdf_xacro.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/hans/ros2_ws/src/ros2_control_demos/example_3/test/test_urdf_xacro.py" "-o" "cache_dir=/home/hans/ros2_ws/build/ros2_control_demo_example_3/ament_cmake_pytest/example_3_urdf_xacro/.cache" "--junit-xml=/home/hans/ros2_ws/build/ros2_control_demo_example_3/test_results/ros2_control_demo_example_3/example_3_urdf_xacro.xunit.xml" "--junit-prefix=ros2_control_demo_example_3")
set_tests_properties(example_3_urdf_xacro PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/hans/ros2_ws/build/ros2_control_demo_example_3" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/hans/ros2_ws/src/ros2_control_demos/example_3/CMakeLists.txt;68;ament_add_pytest_test;/home/hans/ros2_ws/src/ros2_control_demos/example_3/CMakeLists.txt;0;")
add_test(view_example_3_launch "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/hans/ros2_ws/build/ros2_control_demo_example_3/test_results/ros2_control_demo_example_3/view_example_3_launch.xunit.xml" "--package-name" "ros2_control_demo_example_3" "--output-file" "/home/hans/ros2_ws/build/ros2_control_demo_example_3/ament_cmake_pytest/view_example_3_launch.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/hans/ros2_ws/src/ros2_control_demos/example_3/test/test_view_robot_launch.py" "-o" "cache_dir=/home/hans/ros2_ws/build/ros2_control_demo_example_3/ament_cmake_pytest/view_example_3_launch/.cache" "--junit-xml=/home/hans/ros2_ws/build/ros2_control_demo_example_3/test_results/ros2_control_demo_example_3/view_example_3_launch.xunit.xml" "--junit-prefix=ros2_control_demo_example_3")
set_tests_properties(view_example_3_launch PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/hans/ros2_ws/build/ros2_control_demo_example_3" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/hans/ros2_ws/src/ros2_control_demos/example_3/CMakeLists.txt;69;ament_add_pytest_test;/home/hans/ros2_ws/src/ros2_control_demos/example_3/CMakeLists.txt;0;")
add_test(run_example_3_launch "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/hans/ros2_ws/build/ros2_control_demo_example_3/test_results/ros2_control_demo_example_3/run_example_3_launch.xunit.xml" "--package-name" "ros2_control_demo_example_3" "--output-file" "/home/hans/ros2_ws/build/ros2_control_demo_example_3/ament_cmake_pytest/run_example_3_launch.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/hans/ros2_ws/src/ros2_control_demos/example_3/test/test_rrbot_system_multi_interface_launch.py" "-o" "cache_dir=/home/hans/ros2_ws/build/ros2_control_demo_example_3/ament_cmake_pytest/run_example_3_launch/.cache" "--junit-xml=/home/hans/ros2_ws/build/ros2_control_demo_example_3/test_results/ros2_control_demo_example_3/run_example_3_launch.xunit.xml" "--junit-prefix=ros2_control_demo_example_3")
set_tests_properties(run_example_3_launch PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/hans/ros2_ws/build/ros2_control_demo_example_3" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/hans/ros2_ws/src/ros2_control_demos/example_3/CMakeLists.txt;70;ament_add_pytest_test;/home/hans/ros2_ws/src/ros2_control_demos/example_3/CMakeLists.txt;0;")
