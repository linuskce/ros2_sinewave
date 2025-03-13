cmake_minimum_required(VERSION 3.10)
project(ros2_sinewave_interfaces)

# Find required packages.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces (services, messages, etc.).
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConvertImage.srv"
)

# Export dependencies for the generated interfaces.
ament_export_dependencies(rosidl_default_runtime)

# Declare that this package is a ROS 2 interface package.
ament_package()
