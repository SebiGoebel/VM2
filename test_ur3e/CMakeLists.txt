add_executable(cancel_action_ur3e cancel_action_ur3e.cpp)                               # geändert von cancel_action auf cancel_action_ur3e
ament_target_dependencies(cancel_action_ur3e ${THIS_PACKAGE_INCLUDE_DEPENDS})           # geändert von cancel_action auf cancel_action_ur3e
target_link_libraries(cancel_action_ur3e ${LIBRARIES})                                  # geändert von cancel_action auf cancel_action_ur3e

#add_executable(hybrid_planning_demo_node hybrid_planning_demo_node.cpp)
#ament_target_dependencies(hybrid_planning_demo_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
#target_link_libraries(hybrid_planning_demo_node ${LIBRARIES})

add_executable(hybrid_planning_ur3e_node hybrid_planning_ur3e_node.cpp)                 # hinzugefügt
ament_target_dependencies(hybrid_planning_ur3e_node ${THIS_PACKAGE_INCLUDE_DEPENDS})    # hinzugefügt
target_link_libraries(hybrid_planning_ur3e_node ${LIBRARIES})                           # hinzugefügt

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  # TODO (vatanaksoytezer / andyze: Flaky behaviour, investigate and re-enable this test asap)
  # Basic integration tests
  # ament_add_gtest_executable(test_basic_integration
  #     test_basic_integration.cpp
  # )
  # ament_target_dependencies(test_basic_integration ${THIS_PACKAGE_INCLUDE_DEPENDS})
  # add_ros_test(launch/test_basic_integration.test.py TIMEOUT 50 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

endif()
