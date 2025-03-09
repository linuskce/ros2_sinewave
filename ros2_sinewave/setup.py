from setuptools import setup, find_packages
from generate_parameter_library_py.setup_helper import generate_parameter_module
import os
from glob import glob

package_name = 'ros2_sinewave'

# Generate the parameter module — no manual file moving
generate_parameter_module(
    module_name=f"{package_name}.sinewave_parameters",
    yaml_file=os.path.join('param', 'sinewave_parameters.yaml'),
    install_base=package_name
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'generate_parameter_library'],
    zip_safe=True,
    maintainer='linus',
    maintainer_email='ge75puk@mytum.de',
    description='ROS2 package for publishing and subscribing to sine wave values',
    license='TODO: License declaration',
    tests_require=['pytest'],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            'sine_publisher = ros2_sinewave.sine_publisher:main',
            'sine_subscriber = ros2_sinewave.sine_subscriber:main',
        ],
    },
)



