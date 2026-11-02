from glob import glob

from setuptools import setup

package_name = 'ocean_sim2sim_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taeung',
    maintainer_email='taeung1203ai@gmail.com',
    description='Isaac Sim / real-robot ROS 2 topics on top of Gazebo ros2_control.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'interface_node = ocean_sim2sim_interface.interface_node:main',
        ],
    },
)
