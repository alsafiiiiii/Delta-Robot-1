cmake_minimum_required(VERSION 3.10)
project(velocity_pub)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)  # Added for JointTrajectory
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(
    DIRECTORY
      scripts
      launch
    DESTINATION
      share/${PROJECT_NAME}/
  )


install(
    PROGRAMS
    scripts/robot_control.py
    scripts/delta_ik_controller.py         # Added
    scripts/delta_position_test.py         # Added
    scripts/delta_trajectory_planner.py  # NEW
    scripts/ee_controller.py  # NEW
    scripts/delta_5dof_controller.py # NEW
    scripts/delta_gcode_interpreter.py
    scripts/gui_controller.py
    scripts/joystick_controller.py
    scripts/camera_pnp.py
    scripts/delta_3dof_controller.py
    scripts/direct_joystick_controller.py
    scripts/star_draw_no_ros.py
    launch/four_ws_control.launch.py
    DESTINATION lib/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
