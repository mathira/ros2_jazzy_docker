#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class SpawnTurtle(Node):
    def __init__(self):
        super().__init__('spawn_turtle')
        self.client = self.create_client(Spawn, '/spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/spawn service not available, waiting...')

        self.req = Spawn.Request()
        self.req.x = 5.0
        self.req.y = 5.0
        self.req.theta = 0.0
        self.req.name = 'turtle2'

    def call(self):
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.done_callback)

    def done_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return
        if response.name:
            self.get_logger().info(f'Spawning {response.name} at (5, 5)')
        else:
            self.get_logger().info('Spawn failed (name already in use?)')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtle()
    node.call()
    rclpy.spin(node)


if __name__ == '__main__':
    main()