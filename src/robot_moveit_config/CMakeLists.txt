cmake_minimum_required(VERSION 3.5)
project(robot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch config srdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/fake_trajectory_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Generate hooks for environment setup including AMENT_PREFIX_PATH
ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
)

ament_package()
