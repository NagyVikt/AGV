from setuptools import find_packages, setup
from glob import glob

package_name = 'agv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # This file is required for ament to find your package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Your package.xml is installed in the share directory
        ('share/' + package_name, ['package.xml']),
        # Install all launch files into share/agv/launch
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deadpool',
    maintainer_email='nagy.viktordp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # If you have executable nodes, list them here.
            # Example: 'my_node = agv.my_node:main'
        ],
    },
)
