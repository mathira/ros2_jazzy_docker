from setuptools import find_packages, setup

package_name = 'turtle_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@example.com',
    description='Python turtlesim teleop and spawn nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_turtle = turtle_py.teleop_turtle:main',
            'spawn_turtle = turtle_py.spawn_turtle:main',
        ],
    },
)