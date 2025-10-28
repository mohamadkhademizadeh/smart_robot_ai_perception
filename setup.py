from setuptools import setup, find_packages

package_name = 'smart_robot_ai_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch', ['smart_robot_ai_perception/launch/sim_perception.launch.py']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 AI perception + fusion + control pipeline',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = smart_robot_ai_perception.nodes.detector_node:main',
            'lidar_node = smart_robot_ai_perception.nodes.lidar_node:main',
            'fusion_node = smart_robot_ai_perception.nodes.fusion_node:main',
            'controller_node = smart_robot_ai_perception.nodes.controller_node:main',
        ],
    },
)
