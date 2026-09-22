"""This node implements a simple state machine to drive the robot in a square pattern. 
The robot will drive straight for a set distance, then turn 90 degrees, and repeat this process for a total of four sides."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DriveSquareNode(Node):
    def __init__(self):
        "This class Initializes the node, sets up the publisher and subscriber, and initializes parameters for driving in a square pattern"
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

        self.state = 'straight' # or turn
        self.state_elapsed_time = 0.0
        self.sides_completed = 0
        self.total_sides = 4
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)
        self.yaw = None 
        self.turn_start_yaw = None
        self.turn_tolerance = 0.02 

    def process_odom(self, msg):
        "Processes the odometry data to determine the current yaw of the robot"
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def angle_diff(a, b):
        """Finds the difference between two angles a and b, and returns a value in the range [-pi, pi]"""
        return math.atan2(math.sin(a - b), math.cos(a - b))
    
    def run_loop(self):
        """ If state is straight, publish a straight message, if turning, publish a turn message. Also make the neato do those things. 
        If its been in that state for long enough, switch to the other state. """
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