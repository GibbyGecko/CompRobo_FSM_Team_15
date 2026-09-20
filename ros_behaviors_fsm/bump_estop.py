from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node 

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__("emergency_stop_node")
        self.create_timer(0.1, self.run_loop)
        self.bump_state = False
        self.sub = self.create_subscription(Bump, "bump", self.bumped, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    def bumped(self,msg):
            self.bump_state = (msg.left_front == 1 or \
                               msg.right_front == 1 or \
                               msg.left_side == 1 or \
                                msg.right_side == 1)
    def run_loop(self):
        vel = Twist()
        if self.bump_state == True:
            vel.linear.x = 0.0
        if self.bump_state == False:
            vel.linear.x = 0.1
        self.publisher.publish(vel)
        

def main(args=None):
    rclpy.init(args=args)      
    node = EmergencyStopNode()  
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
