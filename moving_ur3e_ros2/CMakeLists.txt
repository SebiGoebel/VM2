cmake_minimum_required(VERSION 3.8)
project(moving_ur3e_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# --- moving ur3e to a given pose ---
add_executable(moving_ur3e src/moving_ur3e.cpp)
target_include_directories(moving_ur3e PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moving_ur3e PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moving_ur3e
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

install(TARGETS moving_ur3e
  DESTINATION lib/${PROJECT_NAME})


# --- moving ur3e to a given pose with a box ---
add_executable(moving_ur3e_box src/moving_ur3e_box.cpp)
target_include_directories(moving_ur3e_box PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moving_ur3e_box PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moving_ur3e_box
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
  
install(TARGETS moving_ur3e_box
  DESTINATION lib/${PROJECT_NAME})

# --- moving ur3e to a given pose with a box and constrained workarea ---
add_executable(moving_ur3e_box_constrained src/moving_ur3e_box_constrained.cpp)
target_include_directories(moving_ur3e_box_constrained PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moving_ur3e_box_constrained PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moving_ur3e_box_constrained
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
  
install(TARGETS moving_ur3e_box_constrained
  DESTINATION lib/${PROJECT_NAME})

# --- hybrid planning node ---
add_executable(hybrid_planning_ur3e src/hybrid_planning_ur3e.cpp)
target_include_directories(hybrid_planning_ur3e PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hybrid_planning_ur3e PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hybrid_planning_ur3e
  #"planning_scene_monitor"
  "moveit_ros_planning_interface"
  #"robot_model_loader"
  #"kinematic_constraints"
  #"robot_state"
  #"moveit_msgs"
  "moveit_visual_tools"
  "rclcpp"
)
#ament_target_dependencies(hybrid_planning_ur3e PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS hybrid_planning_ur3e
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)                            # hinzugefügt für hybrid planning
  find_package(ros_testing REQUIRED)                                  # hinzugefügt für hybrid planning
  find_package(Boost REQUIRED COMPONENTS filesystem)                  # hinzugefügt für hybrid planning
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)                                  # hinzugefügt für hybrid planning
  set(ament_cmake_uncrustify_FOUND TRUE)                              # hinzugefügt für hybrid planning
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
