from setuptools import setup, find_packages
from generate_parameter_library_py.setup_helper import generate_parameter_module
import os
from glob import glob
import sys
import shutil

package_name = 'ros2_sinewave'

generate_parameter_module(
    module_name="sinewave_parameters",
    yaml_file=os.path.join('param', 'sinewave_parameters.yaml'),
    merge_install=True,
)

#####################################################################################################################
# The generate_parameter_library_py logic places the generated Python file in`install/Lib/pythonX.Y/site-packages/`. 
# However, on Windows the other Python files are installed to `install/Lib/site-packages/<package_name>/` 
# without the `pythonX.Y` subfolder.
#
# This leads to a mismatch: the main Python nodes end up in one directory while the generated `sinewave_parameters.py`
# ends up in a different directory. Python then cannot import the generated file correctly at runtime.
#
# The workaround below locates the generated parameter file and copies it into our local `ros2_sinewave/` 
# source folder before packaging. That way, all `.py` files end up in the same  directory, 
# ensuring `import ros2_sinewave.sinewave_parameters` works as expected across platforms.
#####################################################################################################################

this_file_dir = os.path.dirname(os.path.abspath(__file__))
workspace_root = os.path.dirname(os.path.dirname(this_file_dir)) 
install_dir = os.path.join(workspace_root, "install") 

def copy_generated_param_file():
    # Figure out Python version for the subdirectory name
    major, minor = sys.version_info[:2]
    python_subdir = f"python{major}.{minor}"  # e.g., "python3.8"

    # Potential directories where the library might have placed the file:
    candidate_dirs = [
        os.path.join(install_dir, "Lib", python_subdir, "site-packages", package_name),
        os.path.join(install_dir, "lib", python_subdir, "site-packages", package_name),
    ]

    for cdir in candidate_dirs:
        candidate_file = os.path.join(cdir, "sinewave_parameters.py")
        if os.path.exists(candidate_file):
            shutil.copy2(candidate_file, os.path.join(package_name, "sinewave_parameters.py"))
            return

copy_generated_param_file()

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





