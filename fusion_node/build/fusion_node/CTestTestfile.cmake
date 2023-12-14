# CMake generated Testfile for 
# Source directory: /home/nisarg/dev_ws/src/fusion_node
# Build directory: /home/nisarg/dev_ws/src/fusion_node/build/fusion_node
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/copyright.xunit.xml" "--package-name" "fusion_node" "--output-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/ament_copyright/copyright.txt" "--command" "/opt/ros/dashing/bin/ament_copyright" "--xunit-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/nisarg/dev_ws/src/fusion_node")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/cppcheck.xunit.xml" "--package-name" "fusion_node" "--output-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/dashing/bin/ament_cppcheck" "--xunit-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/cppcheck.xunit.xml")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/nisarg/dev_ws/src/fusion_node")
add_test(cpplint "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/cpplint.xunit.xml" "--package-name" "fusion_node" "--output-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/ament_cpplint/cpplint.txt" "--command" "/opt/ros/dashing/bin/ament_cpplint" "--xunit-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/nisarg/dev_ws/src/fusion_node")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/lint_cmake.xunit.xml" "--package-name" "fusion_node" "--output-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/dashing/bin/ament_lint_cmake" "--xunit-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/nisarg/dev_ws/src/fusion_node")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/uncrustify.xunit.xml" "--package-name" "fusion_node" "--output-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/dashing/bin/ament_uncrustify" "--xunit-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/nisarg/dev_ws/src/fusion_node")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/xmllint.xunit.xml" "--package-name" "fusion_node" "--output-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/ament_xmllint/xmllint.txt" "--command" "/opt/ros/dashing/bin/ament_xmllint" "--xunit-file" "/home/nisarg/dev_ws/src/fusion_node/build/fusion_node/test_results/fusion_node/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/nisarg/dev_ws/src/fusion_node")