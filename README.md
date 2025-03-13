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

    ros2_sinewave_ws/<br/> 
    ├── ros2_sinewave/<br/> 
    │   ├── launch/<br/> 
    │   │   └── sinewave_launch.py<br/> 
    │   ├── param/<br/> 
    │   │   └── sinewave_parameters.yaml<br/> 
    │   ├── ros2_sinewave/<br/> 
    │   │   ├── __init__.py<br/> 
    │   │   ├── sinewave_parameters.py<br/> 
    │   │   ├── sinewave_publisher.py<br/> 
    │   │   └── sinewave_subscriber.py<br/> 
    │   └── test/<br/> 
    │       └── test_service_callback.py<br/> 
    ├── ros2_sinewave_interfaces/<br/> 
    │   ├── srv/<br/> 
    │   │   └── (...service files...)<br/> 
    │   ├── CMakeLists.txt<br/> 
    │   └── package.xml<br/> #

## Parameter Configuration:

This project uses parameters defined in the parameters.yaml file (located in the param/ folder). You can adjust values such as the publisher frequency, amplitude, angular frequency, and phase by modifying this file. When you build the project, these parameters are automatically loaded into the nodes.   

## Prerequisites:

Note: These instructions have been tested on Windows (e.g., Windows 10/11) with a ROS 2 distribution installed. Commands (like call install\local_setup.bat) are specific to Windows. If you’re on Linux, you’ll need to adapt commands (e.g., use source install/setup.bash) accordingly.

1. **ROS 2 Installation**  
   You need a working ROS 2 environment (e.g., Humble, Iron, Rolling).  
   - [Install ROS 2](https://docs.ros.org/en/rolling/Installation.html) (adjust for your OS/distribution).
   - On Windows, ensure you have Visual Studio and the necessary ROS 2 setup scripts installed.

2. **Colcon**  
   The [colcon build tool](https://colcon.readthedocs.io/en/released/) is required to build your workspace.  
   - On most ROS 2 installations, colcon is already included.  
   - Verify by running:  
     ```bash
     colcon --version
     ```

3. **Python 3**  
   - ROS 2 uses Python 3. Confirm by running:  
     ```bash
     python --version
     ```
   - If you’re on Windows, ensure `python` is in your PATH.

Once you have these prerequisites, follow the [Build Instructions](#build-instructions) below to build and run the project.

## Build Instructions

1. **Clone the repository**:
    ```bash
    git clone https://github.com/linuskce/ros2_sinewave_ws.git

2. **Include Generate Parameter Library**:
    ```bash
    cd ros2_sinewave_ws 
    https://github.com/PickNikRobotics/generate_parameter_library.git

3. **Build Packages and source the setup file**:<br/> 
    Note (Windows):
    Before building or running this project, ensure you have sourced your ROS 2 environment. For example:<br/>
    ```bash
    call C:\dev\ros2_jazzy\ros2-windows\local_setup.bat
    ```
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

6. **Plot the Sine Values**:<br/>  
    In a new Terminal:
    ```bash
    cd ros2_sinewave_ws 
    ros2 run rqt_console rqt_console
    ```
    (The publisher and subscriber values are plotted into one column but you can set a filter, to either only show the publisher or the subscriber)

7. **Call the Custom Service (To convert an RGB image into grayscale)**:<br/> 
    In a new Terminal:
    ```bash
    cd ros2_sinewave_ws 
    ros2 service call /convert_image ros2_sinewave_interfaces/srv/ConvertImage "{image_path: 'C:/absolute/path/to/image.jpg'}"
    ```
    (The generated grayscale image is saved in the same folder as the original image)
8. **Run a Unit Test to Test the Service**:<br/> 
    In a new Terminal:
    ```bash
    cd ros2_sinewave_ws 
    pytest ros2_sinewave/test/test_service_callback.py
    ```












   

