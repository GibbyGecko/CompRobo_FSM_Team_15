""" This node uses the laser scan measurement pointing straight ahead from
    the robot and compares it to a desired set distance.  The forward velocity
    of the robot is adjusted until the robot achieves the desired distance """

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from enum import Enum
import math



class State(Enum):
    """ This enum defines the states of the wall following state machine """
    APPROACH = 1
    FOLLOW = 2
    TURN = 3

class WallFollowingNode(Node):
    """ This class wraps the basic functionality of the node """
    def __init__(self):
        super().__init__('wall_approach')
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # distance_to_obstacle is used to communciate laser data to run_loop
        self.distance_to_obstacle = None
        # Kp is the constant or to apply to the proportional error signal
        self.Kp = 0.5
        # target_distance is the desired distance to the obstacle in front
        self.state = State.APPROACH
        self.ray_offset = 30 #offset from perpendicular for the ray used to detect the wall
        self.target_distance = 0.5
        self.front = None
        self.back = None
        self.side_angle = 270
        self.follow_speed = 0.3
        self.follow_distance = 0.5
        self.Kp_angle = 1.0
        self.Kp_distance = 0.5
        self.parralel_tolerance = 0.05 # offset from parralel to the wall to consider the robot parralel to the wall
        self.approach_tolerance = 0.05
        self.side = 1.0 if self.side_angle == 270 else -1.0 #determines if the robot is following the wall on the left or right side
        self.turn_speed = 0.5


    def run_loop(self):
        msg = Twist()
        if self.state == State.APPROACH:
                    msg = self.do_approach()
        elif self.state == State.TURN:
                    msg = self.do_turn()
        elif self.state == State.FOLLOW:
                    msg = self.do_follow()
        self.vel_pub.publish(msg)

    def do_approach(self):
        msg = Twist()
        if self.distance_to_obstacle is None:
            msg.linear.x = 0.0
            print("No laser data")
            return msg
        error = self.distance_to_obstacle - self.target_distance
        if abs(error) < self.approach_tolerance:
            self.state = State.TURN
            return msg
        msg.linear.x = self.Kp * error
        return msg
              
    def do_turn(self):
        msg = Twist()
        turn_dir = self.side 
        if self.side_front is not None and self.side_back is not None:
              if abs(self.side_front - self.side_back) < self.parralel_tolerance:
                self.state = State.FOLLOW
                return msg 
        msg.angular.z = self.turn_speed * turn_dir
        return msg


    def do_follow(self):
        msg = Twist()
        if self.side_front is None or self.side_back is None:
            msg.linear.x = 0.05
            return msg
        angle_error = self.side_front - self.side_back 
        dist = 0.5 * (self.side_front + self.side_back) * math.cos(math.radians(self.ray_offset))
        distance_error = dist - self.follow_distance
        msg.linear.x = self.follow_speed
        msg.angular.z = self.side * (-self.Kp_angle * angle_error - self.Kp_distance * distance_error)
        return msg

    def process_scan(self, msg):

        r = msg.ranges
        if r[0] != 0.0:
            self.distance_to_obstacle = r[0]
            front_index = int(self.side_angle + self.side * self.ray_offset) % 360
            back_index = int(self.side_angle - self.side * self.ray_offset) % 360
            self.side_front = r[front_index]
            self.side_back = r[back_index]


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
