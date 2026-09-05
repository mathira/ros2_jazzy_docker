from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_package_declares_runtime_dependencies():
    xml = (PACKAGE_ROOT / 'package.xml').read_text()
    for name in (
        'rclpy',
        'sensor_msgs',
        'nav_msgs',
        'geometry_msgs',
        'tf2_ros',
        'ros_gz_interfaces',
    ):
        assert f'<exec_depend>{name}</exec_depend>' in xml
