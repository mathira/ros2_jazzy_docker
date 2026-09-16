from math import cos, sin


def map_to_odom(map_x, map_y, map_yaw, odom_x, odom_y, odom_yaw):
    """Compose map->base_link with inverse odom->base_link poses."""
    yaw = map_yaw - odom_yaw
    return (
        map_x - cos(yaw) * odom_x + sin(yaw) * odom_y,
        map_y - sin(yaw) * odom_x - cos(yaw) * odom_y,
        yaw,
    )
