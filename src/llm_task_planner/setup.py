from glob import glob
from setuptools import find_packages, setup

package_name = 'llm_task_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='LLM-based high-level task planner for ROS 2 mobile robots.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner = llm_task_planner.planner_node:main',
            'executor = llm_task_planner.executor_node:main',
        ],
    },
)
