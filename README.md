# ROS2 Sinewave Package 

This repository contains a ROS 2 application that:
- Publishes a sine wave on a topic
- Subscribes to that topic
- Hosts a custom service for converting images to grayscale

## Contents

- **ros2_sinewave/**: Main Python package (publisher, subscriber, service callback)
- **ros2_sinewave_interfaces/**: Custom service definitions (if you split interfaces)
- **.gitignore**: Ignore rules for build, install, and log folders, among others
- **README.md**: This file

## Build Instructions

1. **Clone the repository**:
    ```bash
    git clone https://github.com/linuskce/ros2_sinewave.git

2. **Include Generate Parameter Library**:
    ```bash
    cd ros2_sinewave_ws 
    https://github.com/PickNikRobotics/generate_parameter_library.git

3. **Build Packages and source the setup file**:
    ```bash
    colcon build --merge-install --packages-skip-regex ".*example.*"
    call install\local_setup.bat

4. **Run the Nodes**:
    ```bash
    ros2 launch ros2_sinewave sinewave_launch.py

5. **Run the Nodes seperately (Optional)**:<br/>  
    In Terminal 1:
    ```bash
    ros2 run ros2_sinewave sinewave_publisher
    ```
    In Terminal 2:
    ```bash
    ros2 run ros2_sinewave sinewave_subscriber

6. **Plot the Sine Values**:
    ```bash
    ros2 run rqt_console rqt_console

7. **Call the Custom Service**:
    ```bash
    ros2 service call /convert_image ros2_sinewave_interfaces/srv/ConvertImage "{image_path: 'C:/absolute/path/to/image.jpg'}"










   

