cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_fsm)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFSMState.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  amr_sweeper_fsm/amr_service_caller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

