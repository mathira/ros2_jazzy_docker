from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'stage_autonomous_nav'


def resource_files(directory):
    return [(os.path.join('share', package_name, directory), files)
            for files in [glob(os.path.join(directory, '*'))]
            if files]


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *resource_files('launch'),
        *resource_files('config'),
        *resource_files('maps'),
        *resource_files('rviz'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stage_autonomous_nav maintainers',
    maintainer_email='maintainer@example.com',
    description='Ground-truth localization and autonomous navigation for Stage.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth_localizer = stage_autonomous_nav.ground_truth_localizer:main',
        ],
    },
)
