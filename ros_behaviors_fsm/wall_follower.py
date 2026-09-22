""" This node includes a simple state machine to follow a wall using laser data. 
The robot will approach the wall, turn to be parralel to the wall, and then follow the wall at a set distance. """

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from enum import Enum
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.duration import Duration


class State(Enum):
    """ This enum defines the states of the wall following state machine """
    APPROACH = 1
    FOLLOW = 2
    TURN = 3

class WallFollowingNode(Node):
    """Initializes the node, sets up the publisher and subscriber, and initializes parameters for the wall following state machine"""
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
        self.target_distance = 0.5 #how far away the robot from the wall the robot will be before it starts turning
        self.side_angle = 270 #default angle of perpindicular ray
        self.follow_speed = 0.3 #how fast(m/s) the robot will move while following the wall
        self.follow_distance = 0.5 #how far away the robot will try to stay from the wall while following
        self.Kp_angle = 1.0 #gain for the angle error while following the wall
        self.Kp_distance = 0.5 #gain for the distance error while following the wall
        self.parralel_tolerance = 0.05 # offset from parralel to the wall to consider the robot parralel to the wall
        self.approach_tolerance = 0.05 # offset from the target distance to consider the robot at the target distance
        self.side = 1.0 if self.side_angle == 270 else -1.0 #determines if the robot is following the wall on the left or right side
        self.turn_speed = 0.5 #angular speed to turn the robot while turning to follow the wall (rads/s)
        self.marker_pub = self.create_publisher(Marker, 'wall_marker', 10)
        self.side_front = None #creates the front ray variable
        self.side_back = None #creates the back ray variable
        self.scan_frame = None
        self.scan_stamp = None 


    def run_loop(self):
        "Calls each state's function and publishes the resulting velocity command to the robot"
        msg = Twist()
        if self.state == State.APPROACH:
                    msg = self.do_approach()
        elif self.state == State.TURN:
                    msg = self.do_turn()
        elif self.state == State.FOLLOW:
                    msg = self.do_follow()
        self.vel_pub.publish(msg)
        self.publish_wall_marker()

    def do_approach(self):
        "Approaches the wall until the robot is at the target distance, then switches to turning state"
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
        "Turns the robot until the front and back rays are parralel to the wall, then switches to the following state"
        msg = Twist()
        turn_dir = self.side 
        if self.side_front is not None and self.side_back is not None:
              if abs(self.side_front - self.side_back) < self.parralel_tolerance:
                self.state = State.FOLLOW
                return msg 
        msg.angular.z = self.turn_speed * turn_dir
        return msg


    def do_follow(self):
        "Follows the wall at the target distance, adjusting to stay parralel to the wall"
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
        "Processes the laser scan data to determine the distance to the wall and the front and back rays"
        r = msg.ranges
        if r[0] != 0.0:
            self.distance_to_obstacle = r[0]
            front_index = int(self.side_angle + self.side * self.ray_offset) % 360
            back_index = int(self.side_angle - self.side * self.ray_offset) % 360
            self.side_front = r[front_index]
            self.side_back = r[back_index]
        self.scan_frame = msg.header.frame_id
        self.scan_stamp = msg.header.stamp

    def publish_wall_marker(self):
        "Publishes a marker to visualize the front and back rays used to follow the wall"
        def valid(r):
            return r is not None and math.isfinite(r) and r > 0.0

        if self.scan_frame is None or not (valid(self.side_front) and valid(self.side_back)):
            return
        front_deg = self.side_angle + self.side * self.ray_offset
        back_deg = self.side_angle - self.side * self.ray_offset

        marker = Marker()
        marker.header.frame_id = self.scan_frame
        marker.header.stamp = self.scan_stamp
        marker.ns = 'wall'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.5).to_msg()
        for deg, r in ((back_deg, self.side_back), (front_deg, self.side_front)):
            p = Point()
            p.x = r * math.cos(math.radians(deg))
            p.y = r * math.sin(math.radians(deg))
            marker.points.append(p)
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
