"""This node implements a finite state machine that controls the behavior of a robot. 
It controls three behaviors: driving in a square pattern, driving in a spiral pattern and stopping to avoid collisions
and following a wall. The FSM transitions between these behaviors based on bump sensor data. If both bump sensors are triggered, 
the FSM switches to wall following mode. An emergency stop behavior is implemented to halt the robot if any bump sensors are activated."""
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
import threading
from ros_behaviors_fsm.drive_square import DriveSquareNode
from ros_behaviors_fsm.spiral_collision_avoidance import SpiralCollisionAvoidanceNode
from ros_behaviors_fsm.wall_follower import WallFollowingNode
from ros_behaviors_fsm.bump_estop import EmergencyStopNode


class State(Enum):
    DRIVE_SQUARE = 1
    SPIRAL = 2
    WALL_FOLLOWING = 3


class BehaviorFSMNode(Node):
    def __init__(self, square, spiral, wall, estop):
        """Initializes the node, sets up the publisher and subscriber, and initializes parameters for the finite state machine"""
        super().__init__('finite_state_controller')
        self.square = square
        self.spiral = spiral
        self.wall = wall
        self.estop = estop

        for node in (square, spiral, wall, estop):
            for timer in node.timers:
                timer.cancel()

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)


        self.left_side = False
        self.right_side = False
        self.create_subscription(Bump, 'bump', self.process_bump, 10)

        self.state = State.DRIVE_SQUARE
        self.create_timer(0.1, self.run_loop)
        self.sim_bump = False
        threading.Thread(target=self.wait_for_enter, daemon=True).start()
        self.get_logger().info('Press Enter to simulate both side bumpers')

    def wait_for_enter(self):
        """Waits for the user to press Enter to simulate both side bumpers being triggered"""
        try:
            while True:
                input()
                self.sim_bump = True
        except EOFError:
            pass

    def process_bump(self, msg):
        """Processes the bump sensor data to determine if the left or right side bumpers are triggered"""
        self.left_side = bool(msg.left_side)
        self.right_side = bool(msg.right_side)

    def set_state(self, new_state):
        """Sets the current state of the FSM to the new state and logs the transition"""
        self.get_logger().info(f'{self.state.name} -> {new_state.name}')
        self.state = new_state

    def run_loop(self):
        """Runs the main loop of the FSM, finding the current state and executing its behavior."""
        sim_bump, self.sim_bump = self.sim_bump, False 
        if self.state == State.SPIRAL and (
                (self.left_side and self.right_side) or sim_bump):
            self.set_state(State.WALL_FOLLOWING)
        
        if self.state == State.SPIRAL and self.left_side and self.right_side:
            self.set_state(State.WALL_FOLLOWING)

        if self.estop.bump_state:
            self.vel_pub.publish(Twist())
            return

        if self.state == State.DRIVE_SQUARE:
            self.square.run_loop()
            if self.square.sides_completed >= self.square.total_sides:
                self.set_state(State.SPIRAL)
        elif self.state == State.SPIRAL:
            self.spiral.run_loop()
        elif self.state == State.WALL_FOLLOWING:
            self.wall.run_loop()


def main(args=None):
    rclpy.init(args=args)
    square = DriveSquareNode()
    spiral = SpiralCollisionAvoidanceNode()
    wall = WallFollowingNode()
    estop = EmergencyStopNode()
    fsm = BehaviorFSMNode(square, spiral, wall, estop)

    nodes = (fsm, square, spiral, wall, estop)
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        fsm.vel_pub.publish(Twist())
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()