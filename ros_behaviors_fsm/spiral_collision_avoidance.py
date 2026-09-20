import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SpiralCollisionAvoidanceNode(Node):

    def __init__(self):
        super().__init__('collision_avoidance_node')

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.run_loop)

        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 10)

        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data)

        self.linear_speed = 0.2      # m/s
        self.start_radius = 0.2      # m
        self.radius_growth = 0.03    # m
        self.spiral_time = 0.0 

        self.stop_distance = 1.0     # m
        self.cone_half_angle = 15    # degrees either side of straight ahead
        self.close_to_wall = False

    def process_scan(self, msg):
        n = len(msg.ranges)
        cone = [msg.ranges[i % n]
                for i in range(-self.cone_half_angle, self.cone_half_angle + 1)]
        valid = [r for r in cone if math.isfinite(r) and r > 0.0]
        self.close_to_wall = bool(valid) and min(valid) <= self.stop_distance

    def run_loop(self):
        vel = Twist()
        if not self.close_to_wall:
            radius = self.start_radius + self.radius_growth * self.spiral_time
            vel.linear.x = self.linear_speed
            vel.angular.z = self.linear_speed / radius
            self.spiral_time += self.timer_period
        self.pub_vel.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = SpiralCollisionAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown() 


if __name__ == '__main__':
    main()