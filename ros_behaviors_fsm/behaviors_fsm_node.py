from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
import threading

from ros_behaviors_fsm.drive_square import DriveSquareNode
from ros_behaviors_fsm.collision_avoidance import CollisionAvoidanceNode
from ros_behaviors_fsm.wall_following import WallFollowingNode
from ros_behaviors_fsm.bump_estop import EmergencyStopNode


class State(Enum):
    DRIVE_SQUARE = 1
    COLLISION_AVOIDANCE = 2
    WALL_FOLLOWING = 3


class BehaviorFSMNode(Node):
    def __init__(self, square, collision, wall, estop):
        super().__init__('behavior_fsm')
        self.square = square
        self.collision = collision
        self.wall = wall
        self.estop = estop

        for node in (square, collision, wall, estop):
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
        try:
            while True:
                input()
                self.sim_bump = True
        except EOFError:
            pass

    def process_bump(self, msg):
        self.left_side = bool(msg.left_side)
        self.right_side = bool(msg.right_side)

    def set_state(self, new_state):
        self.get_logger().info(f'{self.state.name} -> {new_state.name}')
        self.state = new_state

    def run_loop(self):
        sim_bump, self.sim_bump = self.sim_bump, False 
        if self.state == State.COLLISION_AVOIDANCE and (
                (self.left_side and self.right_side) or sim_bump):
            self.set_state(State.WALL_FOLLOWING)
        
        if self.state == State.COLLISION_AVOIDANCE and self.left_side and self.right_side:
            self.set_state(State.WALL_FOLLOWING)

        if self.estop.bump_state:
            self.vel_pub.publish(Twist())
            return

        if self.state == State.DRIVE_SQUARE:
            self.square.run_loop()
            if self.square.sides_completed >= self.square.total_sides:
                self.set_state(State.COLLISION_AVOIDANCE)
        elif self.state == State.COLLISION_AVOIDANCE:
            self.collision.run_loop()
        elif self.state == State.WALL_FOLLOWING:
            self.wall.run_loop()


def main(args=None):
    rclpy.init(args=args)
    square = DriveSquareNode()
    collision = CollisionAvoidanceNode()
    wall = WallFollowingNode()
    estop = EmergencyStopNode()
    fsm = BehaviorFSMNode(square, collision, wall, estop)

    nodes = (fsm, square, collision, wall, estop)
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