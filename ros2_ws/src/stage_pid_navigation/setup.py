from glob import glob
import os

from setuptools import find_packages, setup


package_name = "stage_pid_navigation"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("stage_pid_navigation", "launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="stage_pid_navigation maintainers",
    maintainer_email="maintainer@example.com",
    description="PID goal navigation for Stage simulations.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pid_navigator = stage_pid_navigation.pid_navigator:main",
        ],
    },
)
