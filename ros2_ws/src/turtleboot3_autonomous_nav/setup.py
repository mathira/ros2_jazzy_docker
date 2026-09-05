from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'turtleboot3_autonomous_nav'


def resource_files(directory):
    files = glob(os.path.join(directory, '*'))
    if not files:
        return []
    return [(os.path.join('share', package_name, directory), files)]


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *resource_files('launch'),
        *resource_files('config'),
        *resource_files('rviz'),
        *resource_files('models'),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='turtleboot3_autonomous_nav maintainers',
    maintainer_email='maintainer@example.com',
    description='Autonomous TurtleBot3 coverage mapping and DQN exploration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_mapper = turtleboot3_autonomous_nav.coverage_mapper:main',
            'observation_builder = turtleboot3_autonomous_nav.observation_builder:main',
            'safe_motion_controller = turtleboot3_autonomous_nav.safe_motion_controller:main',
            'dqn_explorer = turtleboot3_autonomous_nav.dqn_explorer:main',
            'dqn_trainer = turtleboot3_autonomous_nav.dqn_trainer:main',
        ],
    },
)
