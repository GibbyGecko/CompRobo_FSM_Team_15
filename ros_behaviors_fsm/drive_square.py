""" Drive the Neato in a square, open-loop """
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)

            self.state_elapsed_time += self.timer_period
            if self.state_elapsed_time >=self.turn_angle_time:
                self.state = 'straight'
                self.state_elapsed_time = 0.0
                self.sides_completed += 1
                print('turn done')

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()