""" Drive the Neato in a square, open-loop """
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.side_length = 1.0  # meters
        self.linear_speed = 0.2 #m/s
        self.angular_speed = 0.2 #rad/s
        self.turn_angle = math.pi/2 # 90 degrees in radians
        self.straight_time = self.side_length / self.linear_speed
        self.turn_angle_time = self.turn_angle / self.angular_speed

        self.state = 'straight' # or 'turn'
        self.state_elapsed_time = 0.0
        self.sides_completed = 0
        self.total_sides = 4
        # state tracking, ie straight or turning, and how long we've been in that state, and which side we are on
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)
        self.yaw = None              # latest heading from odometry (rad)
        self.turn_start_yaw = None   # heading when the current turn began
        self.turn_tolerance = 0.02   # rad, about 1 degree

    def process_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def angle_diff(a, b):
        """ a - b, wrapped to [-pi, pi] """
        return math.atan2(math.sin(a - b), math.cos(a - b))
    
    def run_loop(self):
        """ if state is straight, publish a straight message, if turning, publish a turn message. also make the neato do those things. 
        if its been in that state for long enough, switch to the other state. """
        msg = Twist()

        if self.sides_completed >= self.total_sides:
            self.publisher.publish(Twist())
            return
        if self.state == 'straight':
            msg.linear.x = self.linear_speed
            self.publisher.publish(msg)

            self.state_elapsed_time += self.timer_period
            if self.state_elapsed_time >= self.straight_time:
                self.state = 'turn'
                self.state_elapsed_time = 0.0
                print("side done, turning")
        elif self.state == 'turn':
            if self.yaw is None:      
                self.publisher.publish(msg)
                return
            if self.turn_start_yaw is None:
                self.turn_start_yaw = self.yaw
            turned = self.angle_diff(self.yaw, self.turn_start_yaw)
            remaining = self.turn_angle - turned
            if remaining <= self.turn_tolerance:
                self.state = 'straight'
                self.state_elapsed_time = 0.0
                self.turn_start_yaw = None
                self.sides_completed += 1
                print('turn done')
            else:
                msg.angular.z = min(self.angular_speed, max(0.05, 1.5 * remaining))
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()