from setuptools import setup, find_packages
from generate_parameter_library_py.setup_helper import generate_parameter_module
import os
from glob import glob

package_name = 'ros2_sinewave'

generate_parameter_module(
    module_name="sinewave_parameters",
    yaml_file=os.path.join('param', 'sinewave_parameters.yaml'),
    merge_install=False,
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    entry_points={
        'console_scripts': [
            'sinewave_publisher = ros2_sinewave.sinewave_publisher:main',
            'sinewave_subscriber = ros2_sinewave.sinewave_subscriber:main',
        ],
    },
)





