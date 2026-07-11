from setuptools import setup
import os
from glob import glob

package_name = 'my_ik_solution'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [f'resource/{package_name}']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Student IK solution for 4-DOF ASRS robot arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_node = my_ik_solution.fk_node:main',
            'workspace_node = my_ik_solution.workspace_node:main',
            'ik_node = my_ik_solution.ik_node:main',
        ],
    },
)
