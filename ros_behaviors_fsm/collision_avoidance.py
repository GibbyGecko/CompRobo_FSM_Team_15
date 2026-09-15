import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as NP 
import time
from rclpy.qos import qos_profile_sensor_data



class CollisionAvoidanceNode(Node):

    def __init__(self):
        super().__init__('collision_avoidance_node')

        timer_period = 0.1 # in seconds
        self.timer = self.create_timer(timer_period, self.run_loop)

        #self.bump_state = False

        self.close_to_wall = False

        #creates a publisher to the neado comand velovity topic so you can drive the neato
        #x.linear = linera velocity and z.angular is angular velocity
        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 10)


        self.scan_sub = self.create_subscription(LaserScan, "scan", self.process_scan, 10)

    def process_scan(self, msg):
        self.close_to_wall = msg.ranges[0] <= 1
        #print(msg.ranges[0])



    def run_loop(self):
        # Create a Twist message to describe the robot motion
        vel = Twist()
        #print(self.close_to_wall)
        if self.close_to_wall:
            vel.linear.x = 0.0
        else:
            vel.linear.x = 0.1           
        self.pub_vel.publish(vel)





def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize ROS2 network
    node = CollisionAvoidanceNode()  # Create our node
    rclpy.spin(node)  # Run our node
    rclpy.shutdown()  # If interrupted, gracefully shutdown the ROS2 network


if __name__ == '__main__':
    main()